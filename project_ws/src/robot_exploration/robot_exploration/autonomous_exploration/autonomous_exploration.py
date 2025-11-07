import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
import numpy as np


class ExplorerNode(Node):
    def __init__(self, namespace=""):
        super().__init__('explorer')
        self.get_logger().info("Explorer Node Initialized")

        # Subscriber to the map topic
        self.map_sub = self.create_subscription(
            OccupancyGrid, 'cmap', self.map_callback, 10)

        # Action client for navigation
        ns = self.get_namespace() or namespace
        self.nav_to_pose_client = ActionClient(
            self,
            NavigateToPose,
            f'{ns}/navigate_to_pose' if ns else 'navigate_to_pose'
        )

        # Internal state
        self.visited_frontiers = set()
        self.map_data = None
        self.robot_position = (0, 0)
        self.timer = None
        self.is_exploring = False
        self.current_goal_handle = None

    # --- ✅ Nouvelle méthode : démarrage contrôlé ---
    def start_exploration(self):
        if self.is_exploring:
            self.get_logger().warning("Exploration already running")
            return
        self.get_logger().info("Exploration started")
        self.timer = self.create_timer(5.0, self.explore)
        self.is_exploring = True

    # --- ✅ Nouvelle méthode : arrêt contrôlé ---
    def stop_exploration(self):
        if not self.is_exploring:
            self.get_logger().warning("Exploration not running")
            return

        self.get_logger().info("Exploration stopping...")

        # 1️⃣ Arrêter le timer
        if self.timer is not None:
            try:
                self.destroy_timer(self.timer)
                self.get_logger().info("Timer destroyed ✅")
            except Exception as e:
                self.get_logger().warning(f"Failed to destroy timer: {e}")
            self.timer = None

        self.is_exploring = False

        # 2️⃣ Annuler le goal Nav2 actif
        if self.current_goal_handle is not None:
            self.get_logger().info("Canceling active Nav2 goal...")
            try:
                cancel_future = self.current_goal_handle.cancel_goal_async()
                cancel_future.add_done_callback(
                    lambda _: self.get_logger().info("Nav2 goal canceled ✅"))
            except Exception as e:
                self.get_logger().warning(f"Error canceling Nav2 goal: {e}")
            self.current_goal_handle = None

        self.get_logger().info("Exploration fully stopped 🟢")
    
    def grid_to_world(self, row, col):
        info = self.map_data.info
        x = info.origin.position.x + (col + 0.5) * info.resolution
        y = info.origin.position.y + (info.height - row - 0.5) * info.resolution
        return x, y
    
    # --- Callbacks ROS ---
    def map_callback(self, msg):
        self.map_data = msg
        if not hasattr(self, '_map_logged'):
            self.get_logger().info("Map received")
            self._map_logged = True


    # --- Navigation ---
    def navigate_to(self, x, y):
        ns = self.get_namespace()
        goal_msg = PoseStamped()
        goal_msg.header.frame_id = 'map'
        goal_msg.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.position.x = x
        goal_msg.pose.position.y = y
        goal_msg.pose.orientation.w = 1.0

        nav_goal = NavigateToPose.Goal()
        nav_goal.pose = goal_msg

        self.get_logger().info(f"Navigating to goal: x={x:.2f}, y={y:.2f}")

        self.nav_to_pose_client.wait_for_server()
        send_goal_future = self.nav_to_pose_client.send_goal_async(nav_goal)
        send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warning("Goal rejected!")
            return
        self.get_logger().info("Goal accepted")
        self.current_goal_handle = goal_handle
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.navigation_complete_callback)

    def navigation_complete_callback(self, future):
        try:
            result = future.result().result
            self.get_logger().info(f"Navigation completed: {result}")
        except Exception as e:
            self.get_logger().error(f"Navigation failed: {e}")

    # --- Exploration logic ---
    def find_frontiers(self, map_array):
        frontiers = []
        rows, cols = map_array.shape
        for r in range(1, rows - 1):
            for c in range(1, cols - 1):
                if map_array[r, c] == 0:
                    neighbors = map_array[r-1:r+2, c-1:c+2].flatten()
                    if -1 in neighbors:
                        frontiers.append((r, c))
        self.get_logger().info(f"Found {len(frontiers)} frontiers")
        return frontiers

    def choose_frontier(self, frontiers):
        robot_row, robot_col = self.robot_position
        min_distance = float('inf')
        chosen_frontier = None
        for frontier in frontiers:
            if frontier in self.visited_frontiers:
                continue
            distance = np.hypot(robot_row - frontier[0], robot_col - frontier[1])
            if distance < min_distance:
                min_distance = distance
                chosen_frontier = frontier
        if chosen_frontier:
            self.visited_frontiers.add(chosen_frontier)
            self.get_logger().info(f"Chosen frontier: {chosen_frontier}")
        else:
            self.get_logger().warning("No valid frontier found")
        return chosen_frontier

    def explore(self):
        if self.map_data is None:
            self.get_logger().warning("No map data available")
            return

        map_array = np.array(self.map_data.data).reshape(
            (self.map_data.info.height, self.map_data.info.width))

        frontiers = self.find_frontiers(map_array)
        if not frontiers:
            self.get_logger().info("No frontiers found. Exploration complete!")
            self.stop_exploration()  # Arrête automatiquement si tout est exploré
            return

        chosen_frontier = self.choose_frontier(frontiers)
        if not chosen_frontier:
            self.get_logger().warning("No frontiers to explore")
            return

        goal_x, goal_y = self.grid_to_world(chosen_frontier[0], chosen_frontier[1])
        # goal_x = chosen_frontier[1] * self.map_data.info.resolution + self.map_data.info.origin.position.x
        # goal_y = chosen_frontier[0] * self.map_data.info.resolution + self.map_data.info.origin.position.y
        self.navigate_to(goal_x, goal_y)


def main(args=None):
    rclpy.init(args=args)
    node = ExplorerNode()
    try:
        node.start_exploration()
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.stop_exploration()
        node.get_logger().info("Exploration stopped by user")
    finally:
        node.destroy_node()
        rclpy.shutdown()
