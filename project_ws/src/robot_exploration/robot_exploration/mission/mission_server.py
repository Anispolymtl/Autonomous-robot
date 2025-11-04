import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.executors import MultiThreadedExecutor
from limo_interfaces.action import DoMission
import asyncio
import time
from robot_exploration.autonomous_exploration.autonomous_exploration import ExplorerNode


class MissionServer(Node):
    def __init__(self):
        super().__init__("mission_server")

        self._action_server = ActionServer(
            self,
            DoMission,
            "do_mission",
            execute_callback=self.execute_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback,
        )

        ns = self.get_namespace().strip("/")
        self.explorer = ExplorerNode(namespace=ns)

        self.get_logger().info(f"Mission Server up (ns='{ns}')")

    def goal_callback(self, goal_request):
        self.get_logger().info("Received goal request → starting indefinite exploration")
        return GoalResponse.ACCEPT

    def cancel_callback(self, goal_handle):
        self.get_logger().info("DoMission cancel reçu → stopping exploration")
        self.explorer.stop_exploration()
        return CancelResponse.ACCEPT

    def execute_callback(self, goal_handle):
        self.get_logger().info("Mission START (indefinite mode)")
        self.explorer.start_exploration()
        t0 = time.time()

        try:
            while not goal_handle.is_cancel_requested:
                fb = DoMission.Feedback()
                fb.time_elapsed = int(time.time() - t0)
                fb.percent_complete = 0.0
                goal_handle.publish_feedback(fb)
                time.sleep(1.0)  # ← remplace asyncio.sleep
        except Exception as e:
            self.get_logger().error(f"Erreur pendant la mission: {e}")

        # Si la mission est annulée
        if goal_handle.is_cancel_requested:
            self.get_logger().info("Mission canceled by user")
            self.explorer.stop_exploration()

            # on ne renvoie pas goal_handle.canceled()
            result = DoMission.Result()
            result.result_code = 1
            result.result_message = "Mission canceled by user"
            return result

        # Si la mission se termine normalement (cas théorique)
        self.explorer.stop_exploration()
        result = DoMission.Result()
        result.result_code = 0
        result.result_message = "Mission completed successfully"
        self.get_logger().info("Mission SUCCESS")
        return result




def main(args=None):
    rclpy.init(args=args)
    node = MissionServer()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    executor.add_node(node.explorer)
    rclpy.spin(node, executor=executor)
    node.explorer.destroy_node()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
