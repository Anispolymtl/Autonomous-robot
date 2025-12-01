#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from std_srvs.srv import Trigger
from tf2_ros import Buffer, TransformListener
from rclpy.duration import Duration
from tf2_geometry_msgs import do_transform_pose

# Import des messages/services personnalisés
from limo_interfaces.msg import RobotState as RobotStateMsg
from limo_interfaces.srv import SetRobotState


class BaseManager(Node):
    def __init__(self):
        super().__init__('base_manager')

        self.namespace = self.get_namespace().strip("/") or "robot"

        # TF
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.base_pose_odom = None
        self.return_in_progress = False

        # Nav2
        self.nav_action_client = ActionClient(self, NavigateToPose, "navigate_to_pose")

        # Client pour changer l'état du robot
        self.state_client = self.create_client(SetRobotState, '/set_state')
        
        # Attendre que le service soit disponible
        self.get_logger().info("⏳ Attente du State Manager...")
        while not self.state_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("State Manager non disponible, nouvelle tentative...")

        # Subscriptions
        self.create_subscription(Odometry, "odom", self.odom_callback, 10)

        # Service
        self.create_service(Trigger, "return_to_base", self.handle_return_to_base)

        self.get_logger().info(f"✅ [{self.namespace}] BaseManager ready. Waiting for odom...")

    def odom_callback(self, msg):
        if self.base_pose_odom is None:
            self.base_pose_odom = msg.pose.pose
            self.get_logger().info(
                f"📍 [{self.namespace}] Base saved at odom x={self.base_pose_odom.position.x:.2f}, "
                f"y={self.base_pose_odom.position.y:.2f}"
            )

    def set_robot_state(self, state: int):
        """
        Change l'état du robot via le State Manager
        
        Args:
            state: Valeur de l'enum RobotState (ex: RobotStateMsg.RETURN_TO_BASE)
        """
        request = SetRobotState.Request()
        request.state = state
        
        future = self.state_client.call_async(request)
        future.add_done_callback(lambda f: self._state_change_callback(f, state))

    def _state_change_callback(self, future, state):
        """Callback pour confirmer le changement d'état"""
        state_names = {
            RobotStateMsg.WAIT: "WAIT",
            RobotStateMsg.EXPLORATION: "EXPLORATION",
            RobotStateMsg.NAVIGATION: "NAVIGATION",
            RobotStateMsg.RETURN_TO_BASE: "RETURN_TO_BASE",
            RobotStateMsg.CUSTOM_MISSION: "CUSTOM_MISSION"
        }
        
        try:
            response = future.result()
            if response.success:
                self.get_logger().info(f"✅ État changé: {state_names.get(state, state)}")
            else:
                self.get_logger().warn(f"⚠️ Échec changement d'état: {response.message}")
        except Exception as e:
            self.get_logger().error(f"❌ Erreur changement d'état: {e}")

    def handle_return_to_base(self, request, response):

        if self.base_pose_odom is None:
            response.success = False
            response.message = "Base not known."
            return response

        if self.return_in_progress:
            response.success = False
            response.message = "Return already running."
            return response

        # PoseStamped en ODOM
        base_odom_stamped = PoseStamped()
        base_odom_stamped.header.stamp = self.get_clock().now().to_msg()
        base_odom_stamped.header.frame_id = f"{self.namespace}/odom"
        base_odom_stamped.pose = self.base_pose_odom

        # ---- TF LOOKUP ----
        try:
            transform = self.tf_buffer.lookup_transform(
                f"{self.namespace}/map",
                f"{self.namespace}/odom",
                rclpy.time.Time(),
                timeout=Duration(seconds=1.0)
            )
        except Exception as e:
            self.get_logger().error(f"❌ [{self.namespace}] TF lookup failed: {e}")
            response.success = False
            response.message = "TF lookup failed"
            return response

        # ---- TRANSFORM POSE ----
        try:
            base_map_pose = do_transform_pose(base_odom_stamped.pose, transform)
        except Exception as e:
            self.get_logger().error(f"❌ [{self.namespace}] Pose transform failed: {e}")
            response.success = False
            response.message = "Pose transform failed"
            return response

        # Reconstruire un PoseStamped en frame 'map'
        base_map_stamped = PoseStamped()
        base_map_stamped.header.frame_id = f"{self.namespace}/map"
        base_map_stamped.header.stamp = self.get_clock().now().to_msg()
        base_map_stamped.pose = base_map_pose

        # ---- CHANGER L'ÉTAT → RETURN_TO_BASE ----
        self.set_robot_state(RobotStateMsg.RETURN_TO_BASE)

        # ---- NAV2 GOAL ----
        goal = NavigateToPose.Goal()
        goal.pose = base_map_stamped

        self.get_logger().info(
            f"🏠 [{self.namespace}] Returning to base at MAP x={goal.pose.pose.position.x:.2f}, "
            f"y={goal.pose.pose.position.y:.2f}"
        )

        self.return_in_progress = True
        send_future = self.nav_action_client.send_goal_async(goal)
        send_future.add_done_callback(self.goal_response_callback)

        response.success = True
        response.message = "Return-to-base sent."
        return response

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn(f"⚠️ [{self.namespace}] Return-to-base goal REJECTED")
            self.return_in_progress = False
            # Retour à l'état WAIT si rejeté
            self.set_robot_state(RobotStateMsg.WAIT)
            return

        self.get_logger().info(f"✅ [{self.namespace}] Goal accepted.")
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.result_callback)

    def result_callback(self, future):
        self.return_in_progress = False
        
        try:
            result = future.result().result
            self.get_logger().info(f"🎯 [{self.namespace}] Arrived at base. Nav2 result: {result}")
            
            # ---- CHANGER L'ÉTAT → WAIT ----
            self.set_robot_state(RobotStateMsg.WAIT)
            
        except Exception as e:
            self.get_logger().error(f"❌ [{self.namespace}] Nav2 result error: {e}")
            # En cas d'erreur, retour à WAIT aussi
            self.set_robot_state(RobotStateMsg.WAIT)


def main(args=None):
    rclpy.init(args=args)
    node = BaseManager()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()