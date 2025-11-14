import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from std_msgs.msg import Bool
import numpy as np


class ExplorerNode(Node):
    def __init__(self, namespace=""):
        super().__init__('explorer')
        self.get_logger().info("Explorer Node Initialized")

        self.namespace = self.get_namespace().strip("/") or namespace

        # --- Subscribers / Action Clients ---
        self.map_sub = self.create_subscription(
            OccupancyGrid, 'map', self.map_callback, 10)

        self.nav_to_pose_client = ActionClient(
            self, NavigateToPose, 'navigate_to_pose'
        )

        self.resume_sub = self.create_subscription(
            Bool, 'explore/resume', self.resume_callback, 10
        )

        # --- États internes ---
        self.visited_frontiers = set()
        self.map_data = None
        self.robot_position = (0, 0)
        self.timer = None
        self.is_exploring = False
        self.current_goal_handle = None

        # --- Attente des dépendances critiques ---
        self.get_logger().info("⏳ Attente du serveur Nav2...")
        while not self.nav_to_pose_client.wait_for_server(timeout_sec=2.0):
            self.get_logger().warn("Nav2 non prêt, nouvelle tentative dans 2s...")

        self.get_logger().info("⏳ Attente d'une carte sur /map...")
        while rclpy.ok() and self.map_data is None:
            rclpy.spin_once(self, timeout_sec=1.0)
            self.get_logger().warn("Aucune carte reçue, en attente...")

        self.get_logger().info("✅ Toutes les dépendances sont prêtes — démarrage de l'exploration contrôlable.")


    # --------------------------------------------------------------
    #  Callback pour gérer le démarrage/arrêt via /explore/resume
    # --------------------------------------------------------------
    def resume_callback(self, msg: Bool):
        """Active ou désactive l'exploration selon la valeur reçue."""
        if msg.data:
            if not self.is_exploring:
                self.get_logger().info("🟢 Reprise de l'exploration via /explore/resume")
                self.start_exploration()
            else:
                self.get_logger().info("Exploration déjà en cours")
        else:
            if self.is_exploring:
                self.get_logger().info("⛔ Arrêt de l'exploration via /explore/resume")
                self.stop_exploration()
            else:
                self.get_logger().info("Exploration déjà stoppée")

    # --------------------------------------------------------------
    #  Démarrage / arrêt interne
    # --------------------------------------------------------------
    def start_exploration(self):
        if self.is_exploring:
            self.get_logger().warning("Exploration déjà active")
            return
        self.get_logger().info("Exploration démarrée")
        self.timer = self.create_timer(5.0, self.explore)
        self.is_exploring = True

    def stop_exploration(self):
        if not self.is_exploring:
            self.get_logger().warning("Exploration non active")
            return

        self.get_logger().info("Arrêt de l'exploration...")

        # 1️⃣ Stopper le timer
        if self.timer is not None:
            try:
                self.destroy_timer(self.timer)
                self.get_logger().info("Timer détruit ✅")
            except Exception as e:
                self.get_logger().warning(f"Erreur destruction timer: {e}")
            self.timer = None

        self.is_exploring = False

        # 2️⃣ Annuler la navigation active
        if self.current_goal_handle is not None:
            self.get_logger().info("Annulation du goal Nav2 en cours...")
            try:
                cancel_future = self.current_goal_handle.cancel_goal_async()
                cancel_future.add_done_callback(
                    lambda _: self.get_logger().info("Goal Nav2 annulé ✅"))
            except Exception as e:
                self.get_logger().warning(f"Erreur annulation goal: {e}")
            self.current_goal_handle = None

        self.get_logger().info("Exploration complètement arrêtée 🟢")

    # --------------------------------------------------------------
    #  Outils de conversion et callbacks ROS
    # --------------------------------------------------------------
    def grid_to_world(self, row, col):
        info = self.map_data.info
        x = info.origin.position.x + (col + 0.5) * info.resolution
        y = info.origin.position.y + (info.height - row - 0.5) * info.resolution
        return x, y

    def map_callback(self, msg):
        self.map_data = msg
        if not hasattr(self, '_map_logged'):
            self.get_logger().info("🗺️ Map reçue")
            self._map_logged = True

    # --------------------------------------------------------------
    #  Navigation
    # --------------------------------------------------------------
    def navigate_to(self, x, y):
        goal_msg = PoseStamped()
        goal_msg.header.frame_id = 'map'
        goal_msg.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.position.x = x
        goal_msg.pose.position.y = y
        goal_msg.pose.orientation.w = 1.0

        nav_goal = NavigateToPose.Goal()
        nav_goal.pose = goal_msg

        self.get_logger().info(f"Navigation vers (x={x:.2f}, y={y:.2f})")

        self.nav_to_pose_client.wait_for_server()
        send_goal_future = self.nav_to_pose_client.send_goal_async(nav_goal)
        send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warning("Goal rejeté ❌")
            return
        self.get_logger().info("Goal accepté ✅")
        self.current_goal_handle = goal_handle
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.navigation_complete_callback)

    def navigation_complete_callback(self, future):
        try:
            result = future.result().result
            self.get_logger().info(f"Navigation terminée: {result}")
        except Exception as e:
            self.get_logger().error(f"Erreur navigation: {e}")

    # --------------------------------------------------------------
    #  Exploration principale
    # --------------------------------------------------------------
    def find_frontiers(self, map_array):
        frontiers = []
        rows, cols = map_array.shape
        for r in range(1, rows - 1):
            for c in range(1, cols - 1):
                if map_array[r, c] == 0:
                    neighbors = map_array[r-1:r+2, c-1:c+2].flatten()
                    if -1 in neighbors:
                        frontiers.append((r, c))
        self.get_logger().info(f"Frontières trouvées: {len(frontiers)}")
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
            self.get_logger().info(f"Frontière choisie: {chosen_frontier}")
        else:
            self.get_logger().warning("Aucune frontière valide trouvée")
        return chosen_frontier

    def explore(self):
        if self.map_data is None:
            self.get_logger().warning("Pas encore de map disponible")
            return

        map_array = np.array(self.map_data.data).reshape(
            (self.map_data.info.height, self.map_data.info.width))

        frontiers = self.find_frontiers(map_array)
        if not frontiers:
            self.get_logger().info("Aucune frontière détectée. Exploration terminée ✅")
            self.stop_exploration()
            return

        chosen_frontier = self.choose_frontier(frontiers)
        if not chosen_frontier:
            self.get_logger().warning("Pas de frontière sélectionnable")
            return

        goal_x, goal_y = self.grid_to_world(chosen_frontier[0], chosen_frontier[1])
        self.navigate_to(goal_x, goal_y)


# --------------------------------------------------------------
#  Main
# --------------------------------------------------------------
def main(args=None):
    rclpy.init(args=args)
    node = ExplorerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.stop_exploration()
        node.get_logger().info("Exploration arrêtée par l'utilisateur")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
