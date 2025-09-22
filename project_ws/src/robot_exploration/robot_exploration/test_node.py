
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_srvs.srv import SetBool


class CmdVelPublisher(Node):
    def __init__(self):
        super().__init__('cmd_vel_publisher')
        self.publisher_ = self.create_publisher(Twist, '/limo1/cmd_vel', 10)

        # Etat d'activation contrôlé par service pour s'intégrer à l'architecture
        self.identified = False
        self.create_service(SetBool, 'set_identified', self.set_identified_callback)

        # Timer de publication
        self.timer = self.create_timer(0.5, self.timer_callback)

    def set_identified_callback(self, request, response):
        self.identified = bool(request.data)
        response.success = True
        response.message = 'Identified set'
        self.get_logger().info(f'Identified set to {self.identified}')
        return response

    def timer_callback(self):
        if not self.identified:
            return
        msg = Twist()
        # tour sur place pour identification
        msg.linear.x = 0.0
        msg.angular.z = 0.5
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing: linear.x={msg.linear.x}, angular.z={msg.angular.z}')


def main(args=None):
    rclpy.init(args=args)
    node = CmdVelPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
