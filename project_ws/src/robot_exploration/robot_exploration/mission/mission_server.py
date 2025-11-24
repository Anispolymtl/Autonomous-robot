import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.executors import MultiThreadedExecutor
from limo_interfaces.action import DoMission
from std_srvs.srv import SetBool
from std_msgs.msg import String, Bool
from enum import Enum
import time

# ==========================================================
#  Import optionnel de mission_logic.py (sécurisé)
# ==========================================================
try:
    from mission_logic import (
        on_mission_start,
        on_mission_step,
        on_mission_cancel,
        on_mission_end,
    )
    CUSTOM_LOGIC_AVAILABLE = True
except Exception as e:
    CUSTOM_LOGIC_AVAILABLE = False
    print(f"[MissionServer] Aucune logique custom chargée (mission_logic.py) : {e}")


# ==========================================================
#               ENUM - États de mission
# ==========================================================
class MissionState(Enum):
    WAIT = "En attente"
    EXPLORATION = "Exploration"
    NAVIGATION = "Navigation"


# ==========================================================
#                Mission Server principal
# ==========================================================
class MissionServer(Node):
    def __init__(self):
        super().__init__("mission_server")

        # ---------------------------------------------------------
        # Paramètre ROS2 lu une seule fois (pas dynamique)
        # ---------------------------------------------------------
        self.declare_parameter("use_custom_logic", False)
        self.use_custom_logic = self.get_parameter(
            "use_custom_logic"
        ).get_parameter_value().bool_value

        if self.use_custom_logic:
            self.get_logger().info("🧩 Logique custom ACTIVÉE (use_custom_logic=True)")
        else:
            self.get_logger().info("🧱 Logique custom désactivée (use_custom_logic=False)")

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

        self.resume_pub = self.create_publisher(Bool, "explore/resume", 10)
        self.get_logger().info(f"🛰️ Publication vers explore/resume")

        # Attente du subscriber explore/resume
        self.get_logger().info("⏳ Attente de /explore/resume...")
        start_time = time.time()
        while self.count_subscribers("explore/resume") == 0:
            if time.time() - start_time > 10.0:
                self.get_logger().warn("⚠️ Aucun subscriber /explore/resume détecté (timeout 10s)")
                break
            rclpy.spin_once(self, timeout_sec=0.5)

        # Envoi répété de False pour désactiver exploration au lancement
        msg = Bool()
        msg.data = False
        for _ in range(5):
            self.resume_pub.publish(msg)
            self.get_logger().info("🔒 Exploration désactivée (False publié sur /explore/resume)")
            time.sleep(0.5)

        # Service pour changer mode exploration/navigation
        self.mode_srv = self.create_service(SetBool, "change_mode", self.change_mode_callback)

        self.get_logger().info(f"Mission Server prêt (ns='{self.get_namespace()}') ✅")

        if self.use_custom_logic and CUSTOM_LOGIC_AVAILABLE:
            self.get_logger().info("🔗 mission_logic.py chargé avec succès")
        else:
            self.get_logger().info("ℹ️ Logique custom non utilisée")

    # --------------------------------------------------------
    #       Service : changement mode exploration/navigation
    # --------------------------------------------------------
    def change_mode_callback(self, request, response):
        msg = Bool()

        if request.data:  # True = exploration
            if self.state != MissionState.EXPLORATION:
                msg.data = True
                self.resume_pub.publish(msg)
                self.state = MissionState.EXPLORATION
                response.success = True
                response.message = "Exploration relancée"
                self.get_logger().info("🔄 Passage en mode Exploration")
            else:
                response.success = True
                response.message = "Déjà en mode Exploration"

        else:  # False = navigation
            if self.state != MissionState.NAVIGATION:
                msg.data = False
                self.resume_pub.publish(msg)
                self.state = MissionState.NAVIGATION
                response.success = True
                response.message = "Mode navigation activé"
                self.get_logger().info("🛑 Passage en mode Navigation")
            else:
                response.success = True
                response.message = "Déjà en mode Navigation"

        return response

    # --------------------------------------------------------
    #               Publication d'état
    # --------------------------------------------------------
    def publish_state(self):
        msg = String()
        msg.data = self.state.value
        self.state_pub.publish(msg)

    # --------------------------------------------------------
    #                  ActionServer Callbacks
    # --------------------------------------------------------
    def goal_callback(self, goal_request):
        self.get_logger().info("🎯 Goal reçu → lancement mission")
        return GoalResponse.ACCEPT

    def cancel_callback(self, goal_handle):
        self.get_logger().info("🟥 Annulation reçue → arrêt exploration")
        self.stop_exploration()
        self.state = MissionState.WAIT

        if self.use_custom_logic and CUSTOM_LOGIC_AVAILABLE:
            try:
                on_mission_cancel(self)
            except Exception as e:
                self.get_logger().error(f"[mission_logic] Erreur on_mission_cancel: {e}")

        return CancelResponse.ACCEPT

    # --------------------------------------------------------
    #                  Commandes exploration
    # --------------------------------------------------------
    def start_exploration(self):
        msg = Bool()
        msg.data = True
        self.resume_pub.publish(msg)
        self.state = MissionState.EXPLORATION
        self.get_logger().info("🟢 Exploration activée")

    def stop_exploration(self):
        msg = Bool()
        msg.data = False
        self.resume_pub.publish(msg)
        self.get_logger().info("⛔ Exploration stoppée")

    # --------------------------------------------------------
    #                Exécution principale de mission
    # --------------------------------------------------------
    async def execute_callback(self, goal_handle):
        self.get_logger().info("🚀 Mission START (exploration continue)")
        self.start_exploration()
        t0 = time.time()

        # Hook start
        if self.use_custom_logic and CUSTOM_LOGIC_AVAILABLE:
            try:
                on_mission_start(self)
            except Exception as e:
                self.get_logger().error(f"[mission_logic] Erreur on_mission_start: {e}")

        try:
            while rclpy.ok():
                # Annulation
                if goal_handle.is_cancel_requested:
                    self.get_logger().info("🟥 Mission annulée")
                    self.stop_exploration()
                    self.state = MissionState.WAIT

                    result = DoMission.Result()
                    result.result_code = 1
                    result.result_message = "Mission annulée"
                    goal_handle.canceled()
                    return result

                elapsed = int(time.time() - t0)

                # Hook step (permet fin anticipée)
                if self.use_custom_logic and CUSTOM_LOGIC_AVAILABLE:
                    try:
                        end_message = on_mission_step(self, elapsed)
                        if end_message is not None:
                            self.get_logger().info(f"[mission_logic] Fin personnalisée: {end_message}")

                            try:
                                on_mission_end(self)
                            except:
                                pass

                            self.stop_exploration()
                            self.state = MissionState.WAIT

                            result = DoMission.Result()
                            result.result_code = 0
                            result.result_message = end_message
                            goal_handle.succeed()
                            return result
                    except Exception as e:
                        self.get_logger().error(f"[mission_logic] Erreur on_mission_step: {e}")

                # Feedback
                fb = DoMission.Feedback()
                fb.time_elapsed = elapsed
                fb.percent_complete = 0.0
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

        # Fin normale (rarement utilisée)
        self.stop_exploration()
        self.state = MissionState.WAIT

        if self.use_custom_logic and CUSTOM_LOGIC_AVAILABLE:
            try:
                on_mission_end(self)
            except Exception as e:
                self.get_logger().error(f"[mission_logic] Erreur on_mission_end: {e}")

        result = DoMission.Result()
        result.result_code = 0
        result.result_message = "Mission terminée avec succès"
        self.get_logger().info("✅ Mission SUCCESS")
        goal_handle.succeed()
        return result


# ==========================================================
#                     MAIN ROS2
# ==========================================================
def main(args=None):
    rclpy.init(args=args)
    node = MissionServer()

    executor = MultiThreadedExecutor()
    executor.add_node(node)

    try:
        rclpy.spin(node, executor=executor)
    except KeyboardInterrupt:
        node.get_logger().info("Mission Server interrompu (Ctrl+C)")
        node.stop_exploration()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
