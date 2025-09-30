import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger
from geometry_msgs.msg import Twist
import time

class IdentifyRobotService(Node):

    def __init__(self):
        super().__init__('identify_robot_service')
        self.srv = self.create_service(Trigger, 'identify_robot', self.identify_callback)
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)
        self.routine_started = False
        self.routine_duration = 1 # seconde

    def identify_callback(self, request, response):
        if not self.routine_started:
            self.get_logger().info("Declanchement de la routine d'identification")

            try:
                self.routine_started = True
                self.routine()
                response.success = True
                response.message = f"Fin de la routine de {self.get_namespace()}"
            
            except Exception as e:
                # Capture toute erreur et renvoie une réponse négative
                self.get_logger().error(f"Erreur lors du lancement de test_node: {e}")
                response.success = False
                response.message = f"Échec du lancement de la routine : {e}"
        
        else :  
            response.success = False
            response.message = "Échec du lancement de la routine : Routine en cours d'execution!"

        self.get_logger().info(response.message)
        return response

    def routine(self):

        twist = Twist()
        twist.linear.x = 1.0
        twist.angular.z = 0.0
        self.publisher_.publish(twist)
        self.get_logger().info("Avance...")
        time.sleep(self.routine_duration)

        twist = Twist()
        twist.linear.x = -1.0
        twist.angular.z = 0.0
        self.publisher_.publish(twist)
        self.get_logger().info("Recule...")
        time.sleep(self.routine_duration)
        
        self.routine_started = False

def main(args=None):
    rclpy.init(args=args)
    node = IdentifyRobotService()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
