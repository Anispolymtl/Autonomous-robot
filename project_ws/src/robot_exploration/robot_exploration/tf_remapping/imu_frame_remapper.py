#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu

class ImuFrameRemapper(Node):
    def __init__(self):
        super().__init__('imu_frame_remapper')

        # 🔹 Récupération automatique du namespace courant
        ns = self.get_namespace().strip('/')
        if ns == '':
            ns = None  # Cas sans namespace

        # 🔹 Déclaration des paramètres (avec fallback dynamique)
        default_input = f'/{ns}/imu' if ns else '/imu'
        default_output = f'/{ns}/imu_fixed' if ns else '/imu_fixed'
        default_frame = f'{ns}/imu_link' if ns else 'imu_link'

        self.declare_parameter('input_topic', default_input)
        self.declare_parameter('output_topic', default_output)
        self.declare_parameter('new_frame', default_frame)

        # 🔹 Lecture des paramètres finaux (permet aussi override dans launch)
        input_topic = self.get_parameter('input_topic').get_parameter_value().string_value
        output_topic = self.get_parameter('output_topic').get_parameter_value().string_value
        self.new_frame = self.get_parameter('new_frame').get_parameter_value().string_value

        # 🔹 Création de la souscription et de la publication
        self.sub = self.create_subscription(Imu, input_topic, self.callback, 10)
        self.pub = self.create_publisher(Imu, output_topic, 10)

        self.get_logger().info(
            f"[{ns if ns else 'no_ns'}] Remappage IMU de {input_topic} → {output_topic} "
            f"avec frame_id = {self.new_frame}"
        )

    def callback(self, msg):
        msg.header.frame_id = self.new_frame
        self.pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = ImuFrameRemapper()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
