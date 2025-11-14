import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String
import json


class RobotPositionMonitor(Node):
    def __init__(self):
        super().__init__("robot_position_monitor")

        # Declare list of robot namespaces (limo1, limo2, etc.)
        self.declare_parameter("robot_namespaces", ["limo1", "limo2"])

        self.namespaces = (
            self.get_parameter("robot_namespaces")
            .get_parameter_value()
            .string_array_value
        )

        self.robot_positions = {}

        # Publisher that outputs all robot positions as a JSON string
        self.positions_pub = self.create_publisher(
            String, "/robot_positions", 10
        )

        # Create all subscriptions dynamically
        for ns in self.namespaces:
            topic = f"/{ns}/odom"
            self.create_subscription(
                Odometry,
                topic,
                lambda msg, ns=ns: self.odom_callback(msg, ns),
                10,
            )
            self.get_logger().info(f"Subscribed to {topic}")

        # Timer to publish combined position data
        self.create_timer(0.2, self.publish_positions)

        self.get_logger().info(
            f"RobotPositionMonitor started for robots: {self.namespaces}"
        )

    def odom_callback(self, msg: Odometry, robot_ns: str):
        position = msg.pose.pose.position
        orientation = msg.pose.pose.orientation

        # Store robot position
        self.robot_positions[robot_ns] = {
            "x": round(position.x, 3),
            "y": round(position.y, 3),
            "z": round(position.z, 3),
            "orientation": {
                "x": round(orientation.x, 4),
                "y": round(orientation.y, 4),
                "z": round(orientation.z, 4),
                "w": round(orientation.w, 4),
            },
        }

    def publish_positions(self):
        """Publish combined robot positions as JSON for the UI or P2P logic."""
        if not self.robot_positions:
            return

        msg = String()
        msg.data = json.dumps(self.robot_positions)
        self.positions_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = RobotPositionMonitor()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()