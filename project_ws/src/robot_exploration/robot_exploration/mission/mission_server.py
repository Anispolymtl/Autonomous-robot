import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.action.server import ServerGoalHandle
from limo_interfaces.action import DoMission
from geometry_msgs.msg import Twist
import time


class MissionServer(Node):
    def __init__(self):
        super().__init__(
            "mission_server",
            automatically_declare_parameters_from_overrides=True
        )
        self._action_server = ActionServer(
            self,
            DoMission,
            "do_mission",
            execute_callback=self.execute_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback,
        )
        ns_list = list(self.get_parameter('robot_namespaces').get_parameter_value().string_array_value)
        if len(ns_list) != 2:
            self.get_logger().error("robot_namespaces parameter must contain exactly two namespaces.")
            raise ValueError("robot_namespaces parameter must contain exactly two namespaces.")
        self.publisher1_ = self.create_publisher(Twist, f'/{ns_list[0]}/cmd_vel', 10)
        self.publisher2_ = self.create_publisher(Twist, f'/{ns_list[1]}/cmd_vel', 10)
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

            twist1 = Twist()
            twist1.linear.x = 0.0
            twist1.angular.z = 5.0
            self.publisher1_.publish(twist1)

            twist2 = Twist()
            twist2.linear.x = 0.0
            twist2.angular.z = -5.0
            self.publisher2_.publish(twist2)
            time.sleep(1)
        
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