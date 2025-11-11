import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from std_srvs.srv import Trigger
import math

class BaseManager(Node):
    def __init__(self):
        super().__init__('base_manager')

        # Déterminer le namespace du robot automatiquement (ex: limo1, limo2)
        self.namespace = self.get_namespace().strip('/')
        if not self.namespace:
            self.namespace = "robot"

        # Subscribers et Action Client
        self.create_subscription(Odometry, 'odom', self.odom_callback, 10)
        self.nav_action_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

        # Service pour demander un retour à la base
        self.create_service(Trigger, 'return_to_base', self.handle_return_to_base)

        self.base_pose = None
        self.get_logger().info(f"[{self.namespace}] BaseManager ready. Waiting for initial odometry...")

    def odom_callback(self, msg):
        # Sauvegarde la position de départ à la première réception
        if self.base_pose is None:
            self.base_pose = msg.pose.pose
            self.get_logger().info(
                f"[{self.namespace}] Base saved at x={self.base_pose.position.x:.2f}, y={self.base_pose.position.y:.2f}"
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

        # Envoie de l'action de navigation
        self.get_logger().info(f"[{self.namespace}] Returning to base at ({goal.pose.position.x:.2f}, {goal.pose.position.y:.2f})")
        self.nav_action_client.wait_for_server()

        send_goal_future = self.nav_action_client.send_goal_async(
            NavigateToPose.Goal(pose=goal)
        )
        send_goal_future.add_done_callback(self.goal_response_callback)

        response.success = True
        response.message = "Returning to base."
        return response

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn(f"[{self.namespace}] Base navigation goal rejected!")
            return
        self.get_logger().info(f"[{self.namespace}] Base navigation goal accepted.")
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.result_callback)

    def result_callback(self, future):
        result = future.result().result
        self.get_logger().info(f"[{self.namespace}] Navigation to base completed. Status={result}")
        # (Optionnel : vérifier distance finale < 0.3m)

def main(args=None):
    rclpy.init(args=args)
    node = BaseManager()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()