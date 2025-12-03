#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from nav2_msgs.action import NavigateToPose
from action_msgs.msg import GoalStatus
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
        self.waiting_stop_exploration = False
        self.pending_return_goal = None
        self.current_state = None

        # Nav2
        self.nav_action_client = ActionClient(self, NavigateToPose, "navigate_to_pose")

        # Client pour changer l'état du robot
        self.state_client = self.create_client(SetRobotState, 'set_state')

        # Écoute de l'état courant pour synchroniser avec l'exploration
        self.create_subscription(RobotStateMsg, "robot_state", self.robot_state_callback, 10)
        
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

    def robot_state_callback(self, msg: RobotStateMsg):
        """Met à jour l'état courant et déclenche le retour dès que l'état RETURN_TO_BASE est reçu."""
        self.current_state = msg.state

        # Si on attend l'arrêt de l'exploration, lancer le retour dès que l'état n'est plus EXPLORATION
        if (
            self.waiting_stop_exploration
            and self.current_state != RobotStateMsg.EXPLORATION
            and not self.return_in_progress
            and self.pending_return_goal is not None
        ):
            self.get_logger().info("✅ Exploration arrêtée, lancement du retour à la base")
            self.waiting_stop_exploration = False
            self._start_return_goal()
            return

        # Démarrer le retour dès que l'état passe à RETURN_TO_BASE
        if self.current_state == RobotStateMsg.RETURN_TO_BASE:
            self.get_logger().info("🚦 État RETURN_TO_BASE reçu, lancement du retour à la base")
            self._request_return_to_base()
            return

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

        success, message = self._request_return_to_base()
        response.success = success
        response.message = message or ""
        return response

    def _request_return_to_base(self):
        """
        Lance un retour à la base en fonction de l'état courant.
        """
        if self.base_pose_odom is None:
            self.get_logger().warn("⚠️ Base inconnue, retour impossible.")
            return False, "Base not known."

        if self.return_in_progress:
            self.get_logger().info("ℹ️ Retour déjà en cours.")
            return False, "Return already running."

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
            return False, "TF lookup failed"

        # ---- TRANSFORM POSE ----
        try:
            base_map_pose = do_transform_pose(base_odom_stamped.pose, transform)
        except Exception as e:
            self.get_logger().error(f"❌ [{self.namespace}] Pose transform failed: {e}")
            return False, "Pose transform failed"

        # Reconstruire un PoseStamped en frame 'map'
        base_map_stamped = PoseStamped()
        base_map_stamped.header.frame_id = f"{self.namespace}/map"
        base_map_stamped.header.stamp = self.get_clock().now().to_msg()
        base_map_stamped.pose = base_map_pose

        # Conserver le goal pour lancement
        goal = NavigateToPose.Goal()
        goal.pose = base_map_stamped
        self.pending_return_goal = goal

        # Si on explore encore, demander d'abord l'arrêt de l'exploration
        if self.current_state == RobotStateMsg.EXPLORATION:
            self.waiting_stop_exploration = True
            self.get_logger().info("⏸ Arrêt de l'exploration avant le retour à la base")
            # Déclenche le passage à RETURN_TO_BASE (StateManager publiera explore/resume=False)
            self.set_robot_state(RobotStateMsg.RETURN_TO_BASE)
            return True, "Return-to-base pending: stopping exploration first."

        # Sinon on peut lancer directement le retour
        self._start_return_goal()
        return True, "Return-to-base sent."

    def _start_return_goal(self):
        """Envoie le goal Nav2 pour retourner à la base et met l'état à jour."""
        if self.pending_return_goal is None:
            self.get_logger().warn("⚠️ Aucun goal de retour en attente.")
            return

        goal = self.pending_return_goal
        self.pending_return_goal = None
        # Placer le flag immédiatement pour éviter les doubles déclenchements via les callbacks d'état
        self.return_in_progress = True

        # ---- CHANGER L'ÉTAT → RETURN_TO_BASE ----
        self.set_robot_state(RobotStateMsg.RETURN_TO_BASE)

        self.get_logger().info(
            f"🏠 [{self.namespace}] Returning to base at MAP x={goal.pose.pose.position.x:.2f}, "
            f"y={goal.pose.pose.position.y:.2f}"
        )

        send_future = self.nav_action_client.send_goal_async(goal)
        send_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn(f"⚠️ [{self.namespace}] Return-to-base goal REJECTED")
            self.return_in_progress = False
            self.pending_return_goal = None
            self.waiting_stop_exploration = False
            # Retour à l'état WAIT si rejeté
            self.set_robot_state(RobotStateMsg.WAIT)
            return

        self.get_logger().info(f"✅ [{self.namespace}] Goal accepted.")
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.result_callback)

    def result_callback(self, future):
        self.return_in_progress = False
        self.pending_return_goal = None
        self.waiting_stop_exploration = False
        
        try:
            result_response = future.result()
        except Exception as e:
            self.get_logger().error(f"❌ [{self.namespace}] Nav2 result error: {e}")
            # Pas de changement d'état tant que Nav2 n'a pas confirmé succès/échec
            return

        status = result_response.status
        result = result_response.result

        status_names = {
            GoalStatus.STATUS_SUCCEEDED: "SUCCEEDED",
            GoalStatus.STATUS_ABORTED: "ABORTED",
            GoalStatus.STATUS_CANCELED: "CANCELED",
        }
        status_label = status_names.get(status, f"status={status}")

        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info(f"🎯 [{self.namespace}] Arrived at base. Nav2 status: {status_label}, result: {result}")
        else:
            self.get_logger().warn(f"⚠️ [{self.namespace}] Return-to-base finished with {status_label}, result: {result}")

        # ---- CHANGER L'ÉTAT → WAIT ----
        self.set_robot_state(RobotStateMsg.WAIT)


def main(args=None):
    rclpy.init(args=args)
    node = BaseManager()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
