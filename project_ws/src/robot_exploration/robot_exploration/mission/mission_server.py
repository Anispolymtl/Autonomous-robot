import os
import signal
import subprocess
import time

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.executors import MultiThreadedExecutor
from limo_interfaces.action import DoMission


# ---------------------------------------------------------------------
# Gestionnaire de processus pour explore_lite
# ---------------------------------------------------------------------
class ExploreLiteManager:
    def __init__(self, namespace: str = ""):
        self.namespace = namespace
        self.process = None

    def start_exploration(self):
        """Lance explore_lite comme sous-processus ROS2."""
        if self.process is not None:
            print(f"[ExploreLite] Exploration déjà en cours pour '{self.namespace}'.")
            return

        print(f"[ExploreLite] Démarrage d’explore_lite pour '{self.namespace}'...")

        # Commande ROS2 pour lancer le noeud d’exploration
        cmd = [
            "ros2", "launch", "explore_lite", "explore.launch.py",
            f"namespace:={self.namespace}",
            "use_sim_time:=true"
        ]

        # Lance le process dans son propre groupe pour pouvoir le tuer proprement
        self.process = subprocess.Popen(cmd, preexec_fn=os.setsid)
        print(f"[ExploreLite] explore_lite lancé (PID {self.process.pid})")

    def stop_exploration(self):
        """Arrête le processus explore_lite s’il est en cours."""
        if self.process is None:
            print(f"[ExploreLite] Aucun processus à arrêter pour '{self.namespace}'.")
            return

        print(f"[ExploreLite] Arrêt d’explore_lite pour '{self.namespace}'...")
        try:
            os.killpg(os.getpgid(self.process.pid), signal.SIGTERM)
            self.process.wait(timeout=5)
            print(f"[ExploreLite] explore_lite arrêté pour '{self.namespace}'.")
        except Exception as e:
            print(f"[ExploreLite] Erreur à l’arrêt: {e}")
        finally:
            self.process = None


# ---------------------------------------------------------------------
# MissionServer ROS2 (ActionServer)
# ---------------------------------------------------------------------
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

        ns = self.get_namespace().strip("/") or ""
        self.explorer = ExploreLiteManager(namespace=ns)

        self.get_logger().info(f"Mission Server up (ns='{ns}')")

    def goal_callback(self, goal_request):
        self.get_logger().info("Received goal request → starting explore_lite mission")
        return GoalResponse.ACCEPT

    def cancel_callback(self, goal_handle):
        self.get_logger().info("Cancel request received → stopping explore_lite")
        self.explorer.stop_exploration()
        return CancelResponse.ACCEPT

    async def execute_callback(self, goal_handle):
        self.get_logger().info("Mission START (explore_lite mode)")
        self.explorer.start_exploration()
        t0 = time.time()

        try:
            while rclpy.ok():
                if goal_handle.is_cancel_requested:
                    self.get_logger().info("Mission canceled by user")
                    self.explorer.stop_exploration()

                    result = DoMission.Result()
                    result.result_code = 1
                    result.result_message = "Mission canceled by user"
                    goal_handle.canceled()
                    return result

                # Envoi de feedback périodique
                fb = DoMission.Feedback()
                fb.time_elapsed = int(time.time() - t0)
                fb.percent_complete = 0.0
                goal_handle.publish_feedback(fb)
                time.sleep(1.0)
        except Exception as e:
            self.get_logger().error(f"Erreur pendant la mission: {e}")

        # Si la mission se termine (explore_lite a fini ou été arrêté)
        self.explorer.stop_exploration()
        result = DoMission.Result()
        result.result_code = 0
        result.result_message = "Mission completed successfully"
        self.get_logger().info("Mission SUCCESS")
        goal_handle.succeed()
        return result


# ---------------------------------------------------------------------
# Main ROS2 node
# ---------------------------------------------------------------------
def main(args=None):
    rclpy.init(args=args)
    node = MissionServer()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    rclpy.spin(node, executor=executor)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
