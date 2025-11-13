import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from std_srvs.srv import Trigger


class BaseManager(Node):
    def __init__(self):
        super().__init__('base_manager')

        # Détection automatique du namespace : limo1, limo2...
        self.namespace = self.get_namespace().strip('/') or 'robot'

        # Souscription à l’odom du robot
        self.create_subscription(Odometry, 'odom', self.odom_callback, 10)

        # Nav2 ActionClient
        self.nav_action_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

        # Service qui permet d'appeler le retour à la base
        self.create_service(Trigger, 'return_to_base', self.handle_return_to_base)

        self.base_pose = None

        self.get_logger().info(f"[{self.namespace}] BaseManager ready. Waiting for odometry...")

    def odom_callback(self, msg):
        # Stocke la pose initiale du robot (base)
        if self.base_pose is None:
            self.base_pose = msg.pose.pose
            self.get_logger().info(
                f"[{self.namespace}] Base position saved at x={self.base_pose.position.x:.2f}, "
                f"y={self.base_pose.position.y:.2f}"
            )

    def handle_return_to_base(self, request, response):

        if self.base_pose is None:
            response.success = False
            response.message = "Base not yet known."
            return response

        goal = PoseStamped()
        goal.header.frame_id = 'map'
        goal.header.stamp = self.get_clock().now().to_msg()
        goal.pose = self.base_pose

        self.get_logger().info(
            f"[{self.namespace}] Returning to base at "
            f"({self.base_pose.position.x:.2f}, {self.base_pose.position.y:.2f})"
        )

        self.nav_action_client.wait_for_server()
        send_goal_future = self.nav_action_client.send_goal_async(
            NavigateToPose.Goal(pose=goal)
        )
        send_goal_future.add_done_callback(self.goal_response_callback)

        response.success = True
        response.message = "Returning to base triggered."
        return response

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn(f"[{self.namespace}] Return-to-base goal rejected.")
            return

        self.get_logger().info(f"[{self.namespace}] Base navigation goal accepted.")
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.result_callback)

    def result_callback(self, future):
        result = future.result().result
        self.get_logger().info(f"[{self.namespace}] Arrived at base (Nav2 result: {result}).")


def main(args=None):
    rclpy.init(args=args)
    node = BaseManager()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()