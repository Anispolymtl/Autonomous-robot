import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped


class RobotPositionMonitor(Node):

    def __init__(self):
        super().__init__("robot_pose_monitor")

        # Paramètres
        self.declare_parameter("robot_id", "")
        self.declare_parameter("rate", 10.0)

        self.robot_id = self.get_parameter("robot_id").get_parameter_value().string_value
        self.rate = float(self.get_parameter("rate").value)

        # Si aucun robot_id donné, on utilise le namespace
        namespace = self.get_namespace().strip("/")
        if self.robot_id == "":
            self.robot_id = namespace

        if self.robot_id == "":
            self.get_logger().fatal("❌ Donne un namespace OU un robot_id.")
            raise RuntimeError("robot_id manquant")

        # Topics
        self.odom_topic = f"/{self.robot_id}/odom"
        self.output_topic = f"/{self.robot_id}/current_pose"

        self.get_logger().info(f"✅ Subscribing to: {self.odom_topic}")
        self.get_logger().info(f"✅ Publishing to: {self.output_topic}")

        # Subscriber odom
        self.odom_sub = self.create_subscription(
            Odometry,
            self.odom_topic,
            self.odom_callback,
            10
        )

        # Publisher pose
        self.pose_pub = self.create_publisher(
            PoseStamped,
            self.output_topic,
            10
        )

        self.last_odom = None
        self.timer = self.create_timer(1.0 / self.rate, self.timer_callback)

    def odom_callback(self, msg: Odometry):
        self.last_odom = msg

    def timer_callback(self):
        if self.last_odom is None:
            return

        pose_stamped = PoseStamped()
        pose_stamped.header.stamp = self.get_clock().now().to_msg()
        pose_stamped.header.frame_id = self.last_odom.header.frame_id

        pose_stamped.pose = self.last_odom.pose.pose

        self.pose_pub.publish(pose_stamped)

        # self.get_logger().info(
        #     f"[{self.robot_id}] Pose from odom → x: {pose_stamped.pose.position.x:.2f}, "
        #     f"y: {pose_stamped.pose.position.y:.2f}",
        #     throttle_duration_sec=2.0   # 1 message max toutes les 2 secondes
        # )


def main(args=None):
    rclpy.init(args=args)
    node = RobotPositionMonitor()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()