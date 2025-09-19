import rclpy
from rclpy.node import Node
from std_srvs.srv import SetBool
import subprocess


class IdentifyRobotService(Node):

    def __init__(self):
        super().__init__('identify_robot_service')
        self.srv = self.create_service(SetBool, 'identify_robot', self.identify_callback)
        self.launch_started = False
        self.launch_process = None

    def identify_callback(self, request, response):
        if request.data and not self.launch_started:
            self.get_logger().info("Requête reçue : lancement de test_node...")

            try:
                # Lance le launch file dans un NOUVEAU processus
                # Ainsi, le LaunchService tourne dans le thread principal de ce processus
                cmd = ['ros2', 'launch', 'robot_exploration', 'test_node.launch.py']
                self.launch_process = subprocess.Popen(cmd)
                #le subprocess.Popen est ce qui permet de faire le sous-processus qui launch le test_node.launch.py

                self.launch_started = True
                response.success = True
                response.message = "test_node lancé avec succès"
                self.get_logger().info("test_node lancé avec succès")
            except Exception as e:
                response.success = False
                response.message = f"Échec du lancement de test_node : {e}"
                self.get_logger().info(response.message)
        return response


def main(args=None):
    rclpy.init(args=args)
    node = IdentifyRobotService()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
