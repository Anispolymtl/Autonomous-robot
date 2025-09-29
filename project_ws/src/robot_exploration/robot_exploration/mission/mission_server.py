import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.action.server import ServerGoalHandle
from rclpy.executors import MultiThreadedExecutor
from limo_interfaces.action import DoMission
from geometry_msgs.msg import Twist
import time

class MissionServer(Node):
    def __init__(self):
        super().__init__(
            "mission_server"
        )
        self._action_server = ActionServer(
            self,
            DoMission,
            "do_mission",
            execute_callback=self.execute_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback,
        )
        self.publisher = self.create_publisher(Twist, 'cmd_vel', 10)
        self.get_logger().info("Mission Server is running.")


    def goal_callback(self, goal_request):
        self.get_logger().info("Received goal request")
        return GoalResponse.ACCEPT
    
    def cancel_callback(self, goal_handle: ServerGoalHandle):
        self.get_logger().info("Received cancel request")
        return CancelResponse.ACCEPT

    async def execute_callback(self, goal_handle: ServerGoalHandle):
        mission_length = goal_handle.request.mission_length
        feedback_msg = DoMission.Feedback()
        feedback_msg.time_elapsed = 0
        feedback_msg.percent_complete = 0.0
        goal_handle.publish_feedback(feedback_msg)
        self.get_logger().info("Executing goal...")
        while mission_length == 0 or feedback_msg.time_elapsed < mission_length:            
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                self.get_logger().info("Goal canceled")
                result = DoMission.Result()
                result.result_code = 1 # 1 = canceled
                result.result_message = "Mission was canceled"
                return result

            # Simulate some work being done
            self.get_logger().info(f"Mission step {feedback_msg.time_elapsed+1}/{mission_length}")

            twist = Twist()
            twist.linear.x = 0.0
            twist.angular.z = 5.0
            self.publisher.publish(twist)
            feedback_msg.time_elapsed += 1
            feedback_msg.percent_complete = (feedback_msg.time_elapsed / mission_length) * 100 if mission_length > 0 else 0.0
            goal_handle.publish_feedback(feedback_msg)
            time.sleep(0.1)
        
        # set goal final state 
        self.publisher.publish(Twist())
        goal_handle.succeed()
        self.get_logger().info("Goal succeeded")

        result = DoMission.Result()
        result.result_code = 0 # 0 = success
        result.result_message = "Mission completed successfully"
        return result

        

def main(args=None):
    rclpy.init(args=args)
    mission_server = MissionServer()
    executor = MultiThreadedExecutor()
    rclpy.spin(mission_server, executor=executor)
    rclpy.shutdown()

if __name__ == "__main__":
    main()  