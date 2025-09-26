import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.action.server import ServerGoalHandle
from limo_interfaces.action import DoMission


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
        self.get_logger().info("Mission Server is running.")

    def goal_callback(self, goal_request):
        self.get_logger().info("Received goal request")
        return GoalResponse.ACCEPT
    
    def cancel_callback(self, goal_handle: ServerGoalHandle):
        self.get_logger().info("Received cancel request")
        return CancelResponse.ACCEPT

    async def execute_callback(self, goal_handle: ServerGoalHandle):
        mission_length = goal_handle.request.mission_length

        #execute turning for mission_length seconds
        self.get_logger().info("Executing goal...")
        for i in range(mission_length):
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                self.get_logger().info("Goal canceled")
                return DoMission.Result()

            # Simulate some work being done
            self.get_logger().info(f"Mission step {i+1}/{mission_length}")
            await rclpy.sleep(1)
        
        # set goal final state 
        goal_handle.succeed()
        self.get_logger().info("Goal succeeded")

        result = DoMission.Result()
        result.result_code = 0 # 0 = success
        result.result_message = "Mission completed successfully"
        return result

        

def main(args=None):
    rclpy.init(args=args)
    mission_server = MissionServer()
    rclpy.spin(mission_server)
    rclpy.shutdown()

if __name__ == "__main__":
    main()  