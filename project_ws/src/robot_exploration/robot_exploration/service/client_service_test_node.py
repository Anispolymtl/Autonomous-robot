import rclpy
from rclpy.node import Node
from std_srvs.srv import SetBool

class IdentifyClient(Node):

    def __init__(self):
        super().__init__('identify_client')
        self.cli = self.create_client(SetBool, 'identify_robot')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting...')
        self.req = SetBool.Request()

    def send_request(self, identify: bool):
        self.req.data = identify
        return self.cli.call_async(self.req)

def main(args=None):
    rclpy.init(args=args)
    node = IdentifyClient()
    future = node.send_request(True)  # On envoie toujours "identifier"
    rclpy.spin_until_future_complete(node, future)
    if future.result() is not None:
        node.get_logger().info(
            f"Réponse: success={future.result().success}, message='{future.result().message}'"
        )
    else:
        node.get_logger().error("Service call failed ")
    rclpy.shutdown()

if __name__ == '__main__':
    main()