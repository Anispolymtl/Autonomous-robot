import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster
from std_msgs.msg import Float64MultiArray

class AdjustableStaticTF(Node):

    def __init__(self):
        super().__init__("adjustable_static_tf")

        # Paramètres
        self.declare_parameter("parent_frame", "merge_map")
        self.declare_parameter("child_frame", "limo1/map")

        self.parent_frame = self.get_parameter("parent_frame").value
        self.child_frame = self.get_parameter("child_frame").value

        self.br = TransformBroadcaster(self)

        # Valeurs par défaut
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0

        # Sub pour update
        self.create_subscription(
            Float64MultiArray,
            "adjust_tf",
            self.update_tf_callback,
            10
        )

        # Timer de broadcast
        self.timer = self.create_timer(0.1, self.publish_tf)

        self.get_logger().info(
            f"Adjustable TF running: {self.parent_frame} → {self.child_frame}"
        )

    def update_tf_callback(self, msg):
        # msg.data = [x, y, yaw]
        self.x = msg.data[0]
        self.y = msg.data[1]
        self.theta = msg.data[2]

        self.get_logger().info(
            f"Updated TF: x={self.x}, y={self.y}, yaw={self.theta}"
        )

    def publish_tf(self):
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = self.parent_frame
        t.child_frame_id = self.child_frame

        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.translation.z = 0.0

        # Yaw → quaternion
        import math
        t.transform.rotation.z = math.sin(self.theta / 2)
        t.transform.rotation.w = math.cos(self.theta / 2)

        self.br.sendTransform(t)

def main(args=None):
    rclpy.init(args=args)
    node = AdjustableStaticTF()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()