import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.executors import MultiThreadedExecutor
from limo_interfaces.action import DoMission
from std_srvs.srv import SetBool
from std_msgs.msg import String, Bool
from enum import Enum
import time


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

        # Publisher ROS2 commun à explore_lite & ExplorerNode
        self.resume_topic = "explore/resume"
        # Juste après la création du publisher explore/resume
        self.resume_pub = self.create_publisher(Bool, "explore/resume", 10)
        self.get_logger().info(f"🛰️ Publication vers {self.resume_topic}")

        # Désactive exploration dès le départ
        self.get_logger().info("⏳ Attente de /explore/resume...")
        start_time = time.time()
        while self.count_subscribers(self.resume_topic) == 0:
            if time.time() - start_time > 10.0:
                self.get_logger().warn("⚠️ Aucun subscriber /explore/resume détecté (timeout 10s)")
                break
            rclpy.spin_once(self, timeout_sec=0.5)

        # --- Envoi répété de False pour bloquer exploration ---
        msg = Bool()
        msg.data = False
        for _ in range(5):  # plusieurs publications espacées
            self.resume_pub.publish(msg)
            self.get_logger().info("🔒 Exploration désactivée (False publié sur /explore/resume)")
            time.sleep(0.5)
        
        # Service pour changer de mode
        self.mode_srv = self.create_service(SetBool, "change_mode", self.change_mode_callback)

        self.get_logger().info(f"Mission Server prêt (ns='{self.get_namespace()}') ✅")

    # --------------------------------------------------------
    #   Changement de mode Exploration / Navigation
    # --------------------------------------------------------
    def change_mode_callback(self, request, response):
        """
        Service pour passer en mode exploration (True) ou navigation (False)
        Compatible explore_lite & ExplorerNode via /explore/resume
        """
        msg = Bool()

        if request.data:  # True = Exploration
            if self.state != MissionState.EXPLORATION:
                msg.data = True
                self.resume_pub.publish(msg)
                self.state = MissionState.EXPLORATION
                response.success = True
                response.message = "Exploration relancée"
                self.get_logger().info("🔄 Passage en mode Exploration (publié sur /explore/resume)")
            else:
                response.success = True
                response.message = "Déjà en mode Exploration"

        else:  # False = Navigation
            if self.state != MissionState.NAVIGATION:
                msg.data = False
                self.resume_pub.publish(msg)
                self.state = MissionState.NAVIGATION
                response.success = True
                response.message = "Exploration arrêtée, mode Navigation actif"
                self.get_logger().info("🛑 Passage en mode Navigation (exploration stoppée)")
            else:
                response.success = True
                response.message = "Déjà en mode Navigation"

        return response

    # ---------------------------------------------------------
    #   Publication de l'état
    # ---------------------------------------------------------
    def publish_state(self):
        msg = String()
        msg.data = self.state.value
        self.state_pub.publish(msg)

    # ---------------------------------------------------------
    #   Callbacks ActionServer
    # ---------------------------------------------------------
    def goal_callback(self, goal_request):
        self.get_logger().info("🎯 Goal reçu → lancement de la mission d’exploration")
        return GoalResponse.ACCEPT

    def cancel_callback(self, goal_handle):
        self.get_logger().info("🟥 Annulation reçue → arrêt exploration")
        self.stop_exploration()
        self.state = MissionState.WAIT
        return CancelResponse.ACCEPT

    # ---------------------------------------------------------
    #   Commandes d'exploration
    # ---------------------------------------------------------
    def start_exploration(self):
        msg = Bool()
        msg.data = True
        self.resume_pub.publish(msg)
        self.state = MissionState.EXPLORATION
        self.get_logger().info("🟢 Exploration activée (topic /explore/resume=True)")

    def stop_exploration(self):
        msg = Bool()
        msg.data = False
        self.resume_pub.publish(msg)
        self.get_logger().info("⛔ Exploration stoppée (topic /explore/resume=False)")

    # ---------------------------------------------------------
    #   Exécution principale de mission
    # ---------------------------------------------------------
    async def execute_callback(self, goal_handle):
        self.get_logger().info("🚀 Mission START (indefinite exploration mode)")
        self.start_exploration()
        t0 = time.time()

        try:
            while rclpy.ok():
                # Gestion annulation
                if goal_handle.is_cancel_requested:
                    self.get_logger().info("🟥 Mission annulée par l'utilisateur")
                    self.stop_exploration()
                    self.state = MissionState.WAIT

                    result = DoMission.Result()
                    result.result_code = 1
                    result.result_message = "Mission annulée par l'utilisateur"
                    goal_handle.canceled()
                    return result

                # Feedback
                fb = DoMission.Feedback()
                fb.time_elapsed = int(time.time() - t0)
                fb.percent_complete = 0.0  # placeholder
                goal_handle.publish_feedback(fb)

                time.sleep(1.0)

        except Exception as e:
            self.get_logger().error(f"Erreur pendant la mission: {e}")
            self.stop_exploration()
            self.state = MissionState.WAIT

            result = DoMission.Result()
            result.result_code = -1
            result.result_message = f"Erreur: {e}"
            goal_handle.abort()
            return result

        # Fin normale
        self.stop_exploration()
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

    try:
        rclpy.spin(node, executor=executor)
    except KeyboardInterrupt:
        node.get_logger().info("Mission Server interrompu par l'utilisateur")
        node.stop_exploration()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
