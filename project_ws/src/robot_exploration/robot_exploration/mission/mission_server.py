import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.executors import MultiThreadedExecutor
from limo_interfaces.action import DoMission
from std_srvs.srv import SetBool
from std_msgs.msg import String
import asyncio
import time
from robot_exploration.autonomous_exploration.autonomous_exploration import ExplorerNode
from enum import Enum


# -----------------------------
#   Enumération des états
# -----------------------------
class MissionState(Enum):
    WAIT = "En attente"
    EXPLORATION = "Exploration"
    NAVIGATION = "Navigation"


# -----------------------------
#   Mission Server principal
# -----------------------------
class MissionServer(Node):
    def __init__(self):
        super().__init__("mission_server")

        # État initial
        self.state = MissionState.WAIT
        self.get_logger().info(f"Mission Server initialisé (état={self.state.value})")

        # Serveur d’action principale
        self._action_server = ActionServer(
            self,
            DoMission,
            "do_mission",
            execute_callback=self.execute_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback,
        )

        # Publisher d’état
        self.state_pub = self.create_publisher(String, "mission_state", 10)
        self.state_timer = self.create_timer(1.0, self.publish_state)

        # Service pour changer de mode (Exploration ↔ Navigation)
        self.mode_srv = self.create_service(SetBool, "change_mode", self.change_mode_callback)

        # Explorer associé
        ns = self.get_namespace().strip("/")
        self.explorer = ExplorerNode(namespace=ns)


        self.get_logger().info(f"Mission Server prêt (ns='{ns}') ✅")

    # --------------------------------------------------------
    #   Changement de mode Exploration / Navigation
    # --------------------------------------------------------
    def change_mode_callback(self, request, response):
        if request.data:  # True = Exploration
            if self.state != MissionState.EXPLORATION:
                self.get_logger().info("🔄 Passage en mode Exploration → relance de l’exploration")
                try:
                    self.explorer.start_exploration()
                    self.state = MissionState.EXPLORATION
                    response.success = True
                    response.message = "Exploration relancée"
                except Exception as e:
                    self.get_logger().error(f"Erreur lors du start_exploration: {e}")
                    response.success = False
                    response.message = f"Erreur start_exploration: {e}"
            else:
                response.success = True
                response.message = "Déjà en mode Exploration"

        else:  # False = Navigation
            if self.state != MissionState.NAVIGATION:
                self.get_logger().info("🛑 Passage en mode Navigation → arrêt de l’exploration")
                try:
                    self.explorer.stop_exploration()
                    self.state = MissionState.NAVIGATION
                    response.success = True
                    response.message = "Exploration arrêtée, mode Navigation actif"
                except Exception as e:
                    self.get_logger().error(f"Erreur lors du stop_exploration: {e}")
                    response.success = False
                    response.message = f"Erreur stop_exploration: {e}"
            else:
                response.success = True
                response.message = "Déjà en mode Navigation"

        return response
    
    #---------------------------------------------------------
    #   State Publisher ROS2 
    #---------------------------------------------------------
    def publish_state(self):
        msg = String()
        msg.data = self.state.value
        self.state_pub.publish(msg)

    # --------------------------------------------------------
    #   Callbacks ROS2 Action
    # --------------------------------------------------------
    def goal_callback(self, goal_request):
        self.get_logger().info("🎯 Goal reçu → lancement de la mission d’exploration")
        return GoalResponse.ACCEPT

    def cancel_callback(self, goal_handle):
        self.get_logger().info("🟥 Annulation reçue → arrêt de l’exploration")
        self.explorer.stop_exploration()
        self.state = MissionState.WAIT
        return CancelResponse.ACCEPT

    # --------------------------------------------------------
    #   Callback principale de la mission
    # --------------------------------------------------------
    async def execute_callback(self, goal_handle):
        self.get_logger().info("🚀 Mission START (indefinite exploration mode)")

        # Démarrer l’exploration
        self.state = MissionState.EXPLORATION
        self.explorer.start_exploration()
        t0 = time.time()

        try:
            while rclpy.ok():
                # Vérifier si la mission est annulée
                if goal_handle.is_cancel_requested:
                    self.get_logger().info("🟥 Mission annulée par l'utilisateur")
                    self.explorer.stop_exploration()
                    self.state = MissionState.WAIT

                    result = DoMission.Result()
                    result.result_code = 1
                    result.result_message = "Mission annulée par l'utilisateur"
                    goal_handle.canceled()
                    return result

                # Synchroniser les modes
                if self.state == MissionState.NAVIGATION and self.explorer.is_exploring:
                    self.get_logger().info("🛑 Arrêt exploration (mode Navigation actif)")
                    self.explorer.stop_exploration()

                elif self.state == MissionState.EXPLORATION and not self.explorer.is_exploring:
                    self.get_logger().info("🔁 Relance exploration (mode Exploration actif)")
                    self.explorer.start_exploration()

                # Envoyer feedback
                fb = DoMission.Feedback()
                fb.time_elapsed = int(time.time() - t0)
                fb.percent_complete = 0.0  # placeholder
                goal_handle.publish_feedback(fb)

                time.sleep(1.0)

        except Exception as e:
            self.get_logger().error(f"Erreur pendant la mission: {e}")
            self.explorer.stop_exploration()
            self.state = MissionState.WAIT

            result = DoMission.Result()
            result.result_code = -1
            result.result_message = f"Erreur pendant la mission: {e}"
            goal_handle.abort()
            return result

        # Mission terminée normalement
        self.explorer.stop_exploration()
        self.state = MissionState.WAIT

        result = DoMission.Result()
        result.result_code = 0
        result.result_message = "Mission terminée avec succès"
        self.get_logger().info("✅ Mission SUCCESS")
        goal_handle.succeed()
        return result


# --------------------------------------------------------
#   Main ROS2
# --------------------------------------------------------
def main(args=None):
    rclpy.init(args=args)
    node = MissionServer()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    executor.add_node(node.explorer)

    try:
        rclpy.spin(node, executor=executor)
    except KeyboardInterrupt:
        node.get_logger().info("Mission Server interrompu par l'utilisateur")
    finally:
        node.explorer.destroy_node()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
