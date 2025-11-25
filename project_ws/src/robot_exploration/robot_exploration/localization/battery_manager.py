import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from std_srvs.srv import Trigger
import random

class BatteryManager(Node):
    def __init__(self):
        super().__init__("battery_manager")

        self.namespace = self.get_namespace().strip("/") or "robot"

        # Niveau initial de batterie
        self.battery_level = 100.0

        # Publie le niveau de batterie
        self.battery_pub = self.create_publisher(Float32, "battery", 10)

        # Client du service return_to_base
        self.return_client = self.create_client(Trigger, "return_to_base")

        # Timer : mise à jour chaque seconde
        self.timer = self.create_timer(1.0, self.update_battery)

        self.low_battery_triggered = False

        self.get_logger().info(f"[{self.namespace}] BatteryManager started.")

    def update_battery(self):
        # Simulation simple d'une décharge
        self.battery_level = max(0.0, self.battery_level - 1)  

        msg = Float32()
        msg.data = self.battery_level
        self.battery_pub.publish(msg)

        self.get_logger().info(f"[{self.namespace}] Battery = {self.battery_level:.1f}%")

        # Si batterie sous 30% —> retour auto
        if self.battery_level <= 30.0 and not self.low_battery_triggered:
            self.low_battery_triggered = True
            self.get_logger().warn(f"[{self.namespace}] LOW BATTERY — Returning to base!")
            self.trigger_return_to_base()

    def trigger_return_to_base(self):
        if not self.return_client.wait_for_service(timeout_sec=2.0):
            self.get_logger().error("return_to_base service unavailable!")
            return

        req = Trigger.Request()
        future = self.return_client.call_async(req)
        future.add_done_callback(self.on_return_response)

    def on_return_response(self, future):
        """
        Callback quand le service return_to_base répond.
        Si ça échoue, on autorise une tentative future
        (par exemple si Nav2 n’était pas encore prêt).
        """
        try:
            result = future.result()
        except Exception as e:
            self.get_logger().error(f"[BatteryManager] Service call failed: {e}")
            self.low_battery_triggered = False
            return

        if result.success:
            self.get_logger().info("[BatteryManager] Return-to-base accepted by BaseManager.")
        else:
            self.get_logger().error(f"[BatteryManager] Return-to-base FAILED: {result.message}")
            # On redonne la possibilité de réessayer plus tard
            self.low_battery_triggered = False


def main(args=None):
    rclpy.init(args=args)
    node = BatteryManager()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()