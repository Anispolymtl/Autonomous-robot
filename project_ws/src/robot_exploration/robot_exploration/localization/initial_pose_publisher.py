import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry

# Essayons d'importer ModelStates seulement si on est en simulation
try:
    from gazebo_msgs.msg import ModelStates
    GAZEBO_AVAILABLE = True
except ImportError:
    GAZEBO_AVAILABLE = False


class UniversalInitialPosePublisher(Node):
    def __init__(self):
        super().__init__('initial_pose_publisher')

        self.declare_parameter('namespace', 'default_namespace')
        self.namespace = self.get_parameter('namespace').get_parameter_value().string_value

        self.publisher = self.create_publisher(
            PoseWithCovarianceStamped, f'/{self.namespace}/initialpose', 10
        )

        self.pose_published = False

        if GAZEBO_AVAILABLE:
            self.get_logger().info(f"[{self.namespace}] Simulation detected → using /gazebo/model_states")
            self.sub = self.create_subscription(
                ModelStates, '/gazebo/model_states', self.model_state_callback, 10
            )
        else:
            self.get_logger().info(f"[{self.namespace}] Real robot detected → using /{self.namespace}/odom")
            odom_topic = f"/{self.namespace}/odom"
            self.sub = self.create_subscription(Odometry, odom_topic, self.odom_callback, 10)

    # 🧩 Simulation callback
    def model_state_callback(self, msg):
        if self.pose_published:
            return
        if self.namespace not in msg.name:
            return
        idx = msg.name.index(self.namespace)
        pose = msg.pose[idx]
        self.publish_initial_pose(pose)

    # 🤖 Robot réel callback
    def odom_callback(self, msg):
        if self.pose_published:
            return
        pose = msg.pose.pose
        self.publish_initial_pose(pose)

    # 🛰️ Publication commune
    def publish_initial_pose(self, pose):
        msg = PoseWithCovarianceStamped()
        msg.header.frame_id = "map"
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose.pose = pose
        msg.pose.covariance = [0.0] * 36
        msg.pose.covariance[0] = 0.25
        msg.pose.covariance[7] = 0.25
        msg.pose.covariance[35] = 0.1

        self.publisher.publish(msg)
        self.pose_published = True
        self.get_logger().info(
            f"[{self.namespace}] Published initial pose: "
            f"x={pose.position.x:.2f}, y={pose.position.y:.2f}"
        )


def main(args=None):
    rclpy.init(args=args)
    node = UniversalInitialPosePublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()