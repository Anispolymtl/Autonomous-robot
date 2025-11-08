import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, GoalResponse, CancelResponse
from rclpy.executors import MultiThreadedExecutor
from limo_interfaces.action import DoMission
import subprocess
import time


class MissionServer(Node):
    def __init__(self):
        super().__init__("mission_server")

        # Serveur d’action
        self._action_server = ActionServer(
            self,
            DoMission,
            "do_mission",
            execute_callback=self.execute_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback,
        )

        # Gestion du processus explore_lite
        self.explore_process = None

        # Namespace dynamique
        self.namespace = self.get_namespace().strip("/") or "limo1"

        self.get_logger().info(f"Mission Server prêt pour namespace '{self.namespace}' ✅")

    # ------------------------------------------------------------------
    #                    Lancer / Arrêter explore_lite
    # ------------------------------------------------------------------

    def start_explore_lite(self):
        """Lance explore_lite/explore.launch.py en sous-processus."""
        try:
            self.get_logger().info(f"Lancement d'explore_lite pour namespace '{self.namespace}'")
            self.explore_process = subprocess.Popen(
                [
                    "ros2", "launch", "explore_lite", "explore.launch.py",
                    f"namespace:={self.namespace}",
                    "use_sim_time:=false"
                ],
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
            )
            self.get_logger().info("explore_lite lancé ✅ (via subprocess)")
        except Exception as e:
            self.get_logger().error(f"Erreur lors du lancement d'explore_lite : {e}")

    def stop_explore_lite(self):
        """Arrête le processus explore_lite."""
        try:
            if self.explore_process and self.explore_process.poll() is None:
                self.get_logger().info("Arrêt du processus explore_lite...")
                self.explore_process.terminate()
                try:
                    self.explore_process.wait(timeout=3)
                except subprocess.TimeoutExpired:
                    self.get_logger().warning("explore_lite ne répond pas, kill forcé.")
                    self.explore_process.kill()
                self.get_logger().info("explore_lite arrêté 🟢")
            else:
                self.get_logger().warning("Aucun processus explore_lite actif.")
            self.explore_process = None
        except Exception as e:
            self.get_logger().error(f"Erreur lors de l'arrêt d'explore_lite : {e}")

    # ------------------------------------------------------------------
    #                       Action Callbacks
    # ------------------------------------------------------------------

    def goal_callback(self, goal_request):
        self.get_logger().info("Goal reçu → lancement de la mission d'exploration autonome")
        return GoalResponse.ACCEPT

    def cancel_callback(self, goal_handle):
        self.get_logger().info("Annulation reçue → arrêt d'explore_lite")
        self.stop_explore_lite()
        return CancelResponse.ACCEPT

    async def execute_callback(self, goal_handle):
        self.get_logger().info(f"Mission START → explore_lite (namespace={self.namespace})")
        self.start_explore_lite()

        start_time = time.time()

        try:
            while rclpy.ok():
                # Gestion de l’annulation par le client
                if goal_handle.is_cancel_requested:
                    self.get_logger().info("Mission annulée par l'utilisateur")
                    self.stop_explore_lite()

                    result = DoMission.Result()
                    result.result_code = 1
                    result.result_message = "Mission canceled by user"
                    goal_handle.canceled()
                    return result

                # Feedback simple
                fb = DoMission.Feedback()
                fb.time_elapsed = int(time.time() - start_time)
                fb.percent_complete = 0.0  # explore_lite ne renvoie pas de progression
                goal_handle.publish_feedback(fb)

                time.sleep(1.0)

        except Exception as e:
            self.get_logger().error(f"Erreur pendant la mission : {e}")

        # Fin de mission : arrêt de explore_lite
        self.stop_explore_lite()
        result = DoMission.Result()
        result.result_code = 0
        result.result_message = "Mission terminée avec succès"
        goal_handle.succeed()
        self.get_logger().info("Mission SUCCESS ✅")
        return result


# ----------------------------------------------------------------------
#                           MAIN PROGRAM
# ----------------------------------------------------------------------

def main(args=None):
    rclpy.init(args=args)
    node = MissionServer()
    executor = MultiThreadedExecutor()
    executor.add_node(node)

    try:
        node.get_logger().info("Mission Server en cours d'exécution (Ctrl+C pour arrêter)")
        executor.spin()  # <-- à la place de rclpy.spin()
    except KeyboardInterrupt:
        node.get_logger().info("Arrêt demandé par l'utilisateur (Ctrl+C)")
    finally:
        node.stop_explore_lite()
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()



if __name__ == "__main__":
    main()
