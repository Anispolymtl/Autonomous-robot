import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.action.server import ServerGoalHandle
from rclpy.executors import MultiThreadedExecutor
from limo_interfaces.action import DoMission
from geometry_msgs.msg import Twist
import time
from robot_exploration.autonomous_exploration.autonomous_exploration import Explorer


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
        # self.publisher = self.create_publisher(Twist, "cmd_vel", 10)
        # self.get_logger().info("Mission Server is running.")

        ns = self.get_namespace().strip("/")
        self.explorer = Explorer(namespace=ns)

        self.get_logger().info(f"Mission Server up (ns='{ns}')")

    def goal_callback(self, goal_request):
        self.get_logger().info("Received goal request")
        return GoalResponse.ACCEPT

    def cancel_callback(self, goal_handle: ServerGoalHandle):
        # self.get_logger().info("Received cancel request")
        # return CancelResponse.ACCEPT

        self.get_logger().info("DoMission cancel reçu")
        # Stopper exploration si en cours
        self.explorer.stop_exploration()
        return CancelResponse.ACCEPT

    async def execute_callback(self, goal_handle: ServerGoalHandle):
        mission_length = goal_handle.request.mission_length
        self.get_logger().info(f"Mission START (length={mission_length}s ; 0 = infini)")

        # 👉 démarre exploration
        self.explorer.start_exploration()

        # Boucle “mission en cours” (feedback simple)
        t0 = time.time()
        try:
            while mission_length == 0 or (time.time() - t0) < mission_length:
                if goal_handle.is_cancel_requested:
                    self.explorer.stop_exploration()
                    goal_handle.canceled()
                    self.get_logger().info("Mission CANCELED")
                    from limo_interfaces.action import DoMission
                    result = DoMission.Result()
                    result.result_code = 1
                    result.result_message = "Mission was canceled"
                    return result

                # Feedback
                from limo_interfaces.action import DoMission
                fb = DoMission.Feedback()
                fb.time_elapsed = int(time.time() - t0)
                fb.percent_complete = (
                    (fb.time_elapsed / mission_length) * 100.0 if mission_length > 0 else 0.0
                )
                goal_handle.publish_feedback(fb)
                await rclpy.sleep(1.0)

        except Exception as e:
            self.get_logger().error(f"Erreur Mission: {e}")
        finally:
            # 👉 stop exploration quand la mission se termine
            self.explorer.stop_exploration()

        goal_handle.succeed()
        self.get_logger().info("Mission SUCCESS")
        from limo_interfaces.action import DoMission
        result = DoMission.Result()
        result.result_code = 0
        result.result_message = "Mission completed successfully"
        return result

def main(args=None):
    rclpy.init(args=args)
    node = MissionServer()
    executor = MultiThreadedExecutor()
    rclpy.spin(node, executor=executor)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()