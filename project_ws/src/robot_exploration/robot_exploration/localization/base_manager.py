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

        # Subscriptions
        self.create_subscription(Odometry, "odom", self.odom_callback, 10)

        # Service
        self.create_service(Trigger, "return_to_base", self.handle_return_to_base)

        self.get_logger().info(f"[{self.namespace}] BaseManager ready. Waiting for odom...")

    def odom_callback(self, msg):
        if self.base_pose_odom is None:
            self.base_pose_odom = msg.pose.pose
            self.get_logger().info(
                f"[{self.namespace}] Base saved at odom x={self.base_pose_odom.position.x:.2f}, "
                f"y={self.base_pose_odom.position.y:.2f}"
            )

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
                "map",
                f"{self.namespace}/odom",
                rclpy.time.Time(),
                timeout=Duration(seconds=1.0)
            )
        except Exception as e:
            self.get_logger().error(f"[{self.namespace}] TF lookup failed: {e}")
            response.success = False
            response.message = "TF lookup failed"
            return response

        # ---- TRANSFORM POSE (Pose, pas PoseStamped) ----
        try:
            base_map_pose = do_transform_pose(base_odom_stamped.pose, transform)
        except Exception as e:
            self.get_logger().error(f"[{self.namespace}] Pose transform failed: {e}")
            response.success = False
            response.message = "Pose transform failed"
            return response

        # Reconstruire un PoseStamped en frame 'map'
        base_map_stamped = PoseStamped()
        base_map_stamped.header.frame_id = "map"
        base_map_stamped.header.stamp = self.get_clock().now().to_msg()
        base_map_stamped.pose = base_map_pose

        # ---- NAV2 GOAL ----
        goal = NavigateToPose.Goal()
        goal.pose = base_map_stamped

        self.get_logger().info(
            f"[{self.namespace}] Returning to base at MAP x={goal.pose.pose.position.x:.2f}, "
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
            self.get_logger().warn(f"[{self.namespace}] Return-to-base goal REJECTED")
            self.return_in_progress = False
            return

        self.get_logger().info(f"[{self.namespace}] Goal accepted.")
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.result_callback)

    def result_callback(self, future):
        self.return_in_progress = False
        try:
            result = future.result().result
            self.get_logger().info(f"[{self.namespace}] Arrived at base. Nav2 result: {result}")
        except Exception as e:
            self.get_logger().error(f"[{self.namespace}] Nav2 result error: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = BaseManager()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()