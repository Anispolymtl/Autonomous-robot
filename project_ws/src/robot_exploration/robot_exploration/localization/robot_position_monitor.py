import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
from tf2_ros import Buffer, TransformListener
from tf2_geometry_msgs import do_transform_pose
from rclpy.duration import Duration


class RobotPoseMonitor(Node):

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

        # Frames
        self.odom_frame = f"{self.robot_id}/odom"
        self.map_frame = f"{self.robot_id}/map"

        # Topics
        self.odom_topic = f"/{self.robot_id}/odom"
        self.output_topic = f"/{self.robot_id}/current_pose"

        self.get_logger().info(f"✅ Subscribing to: {self.odom_topic}")
        self.get_logger().info(f"✅ Publishing to: {self.output_topic}")
        self.get_logger().info(f"✅ Transforming from {self.odom_frame} → {self.map_frame}")

        # TF buffer & listener
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

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

    import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
from tf2_ros import Buffer, TransformListener
from tf2_geometry_msgs import do_transform_pose
from rclpy.duration import Duration


class RobotPoseMonitor(Node):

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

        # Frames
        self.odom_frame = f"{self.robot_id}/odom"
        self.map_frame = f"{self.robot_id}/map"

        # Topics
        self.odom_topic = f"/{self.robot_id}/odom"
        self.output_topic = f"/{self.robot_id}/current_pose"

        self.get_logger().info(f"✅ Subscribing to: {self.odom_topic}")
        self.get_logger().info(f"✅ Publishing to: {self.output_topic}")
        self.get_logger().info(f"✅ Transforming from {self.odom_frame} → {self.map_frame}")

        # TF buffer & listener
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

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

        # Ne pas spam si TF pas prêt
        if not self.tf_buffer.can_transform(
            self.map_frame,
            self.odom_frame,
            rclpy.time.Time(),
            timeout=Duration(seconds=0.1)
        ):
            return

        pose_odom = PoseStamped()
        pose_odom.header.stamp = rclpy.time.Time().to_msg()
        pose_odom.header.frame_id = self.odom_frame
        pose_odom.pose = self.last_odom.pose.pose

        try:
            pose_map = self.tf_buffer.transform(
                pose_odom,
                self.map_frame,
                timeout=Duration(seconds=0.2)
            )
        except Exception:
            return

        self.pose_pub.publish(pose_map)


def main(args=None):
    rclpy.init(args=args)
    node = RobotPoseMonitor()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()


def main(args=None):
    rclpy.init(args=args)
    node = RobotPoseMonitor()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()