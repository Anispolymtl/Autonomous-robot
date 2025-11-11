import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from math import atan2, asin

class RobotPoseMonitor(Node):
    def __init__(self):
        super().__init__('robot_pose_monitor')

        self.declare_parameter('namespace', 'limo1')
        self.namespace = self.get_parameter('namespace').get_parameter_value().string_value

        odom_topic = f"/{self.namespace}/odom"
        self.create_subscription(Odometry, odom_topic, self.odom_callback, 10)

        self.get_logger().info(f"Monitoring robot pose on {odom_topic}")

    def odom_callback(self, msg):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        z = msg.pose.pose.position.z
        q = msg.pose.pose.orientation

        # Conversion quaternion -> yaw
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        yaw = atan2(siny_cosp, cosy_cosp)

        self.get_logger().info(f"[{self.namespace}] x={x:.2f}, y={y:.2f}, yaw={yaw:.2f} rad")

def main(args=None):
    rclpy.init(args=args)
    node = RobotPoseMonitor()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()