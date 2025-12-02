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
#  Import optionnel : custom_mission.py
# ==========================================================
try:
    from robot_exploration.code_editor.custom_mission import (
        on_mission_start,
        on_mission_step,
        on_mission_cancel,
        on_mission_end,
    )
    CUSTOM_LOGIC_AVAILABLE = True
except Exception as e:
    CUSTOM_LOGIC_AVAILABLE = False
    print(f"[MissionServer] Aucune logique custom chargée : {e}")


# ==========================================================
#               ENUM - États de mission
# ==========================================================
class MissionState(Enum):
    WAIT = "En attente"
    EXPLORATION = "Exploration"
    NAVIGATION = "Navigation"
    CUSTOM = "Mission personnalisée"


# ==========================================================
#                Mission Server principal
# ==========================================================
class MissionServer(Node):
    def __init__(self):
        super().__init__("mission_server")

        # Paramètre ROS2
        self.declare_parameter("use_custom_logic", False)
        self.use_custom_logic = self.get_parameter("use_custom_logic") \
                                        .get_parameter_value().bool_value

        # État initial
        if self.use_custom_logic:
            self.get_logger().info("🧩 Logique custom ACTIVÉE")
            self.state = MissionState.CUSTOM
        else:
            self.get_logger().info("🧱 Logique par défaut utilisée")
            self.state = MissionState.WAIT

        self.get_logger().info(f"Mission Server initialisé (état={self.state.value})")

        # Action Server
        self._action_server = ActionServer(
            self, DoMission, "do_mission",
            execute_callback=self.execute_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback,
        )

        # Publishers
        self.state_pub = self.create_publisher(String, "mission_state", 10)
        self.state_timer = self.create_timer(1.0, self.publish_state)

        self.resume_pub = self.create_publisher(Bool, "explore/resume", 10)
        self.get_logger().info("🛰️ Publication vers /explore/resume")

        # Service de mode
        self.mode_srv = self.create_service(SetBool, "change_mode", self.change_mode_callback)

        self.get_logger().info(f"Mission Server prêt (ns='{self.get_namespace()}') ✅")

    # --------------------------------------------------------
    def publish_state(self):
        msg = String()
        msg.data = self.state.value
        self.state_pub.publish(msg)

    # --------------------------------------------------------
    #                 CALL BACKS PRINCIPAUX
    # --------------------------------------------------------
    # --------------------------------------------------------
    def change_mode_callback(self, request, response):
        """
        Service pour passer en mode exploration (True)
        ou navigation (False) en publiant sur /explore/resume
        """
        if self.use_custom_logic:
            response.success = False
            response.message = "Impossible de changer de mode en mission personnalisée"
            return response
        
        msg = Bool()

        if request.data:  # True = Exploration
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

        else:  # False = Navigation
            if self.state != MissionState.NAVIGATION:
                msg.data = False
                self.resume_pub.publish(msg)
                self.state = MissionState.NAVIGATION
                response.success = True
                response.message = "Mode Navigation activé"
                self.get_logger().info("🛑 Passage en mode Navigation")
            else:
                response.success = True
                response.message = "Déjà en mode Navigation"

        return response

    def goal_callback(self, goal_request):
        self.get_logger().info("🎯 Goal reçu → lancement mission")
        return GoalResponse.ACCEPT

    def cancel_callback(self, goal_handle):
        self.get_logger().info("🟥 Annulation reçue")
        self.stop_exploration()
        self.state = MissionState.WAIT

        # --- Custom cancel ---
        if self.use_custom_logic and CUSTOM_LOGIC_AVAILABLE:
            try:
                on_mission_cancel(self)
            except Exception as e:
                self.get_logger().error(f"[custom] Erreur on_mission_cancel: {e}")

        return CancelResponse.ACCEPT

    # --------------------------------------------------------
    def start_exploration(self):
        if self.use_custom_logic:
            self.state = MissionState.CUSTOM
            return
        self.resume_pub.publish(Bool(data=True))
        self.state = MissionState.EXPLORATION
        self.get_logger().info("🟢 Exploration activée (/explore/resume=True)")

    def stop_exploration(self):
        if self.use_custom_logic:
            self.state = MissionState.CUSTOM
            return
        self.resume_pub.publish(Bool(data=False))
        self.get_logger().info("⛔ Exploration stoppée (/explore/resume=False)")

    # --------------------------------------------------------
    #                  EXECUTION PRINCIPALE
    # --------------------------------------------------------
    async def execute_callback(self, goal_handle):
        t0 = time.time()

        self.get_logger().info("🚀 Mission START")

        # ===============================
        #     MODE CUSTOM ACTIVÉ ?
        # ===============================
        if self.use_custom_logic and CUSTOM_LOGIC_AVAILABLE:
            self.state = MissionState.CUSTOM 
            self.get_logger().info("🧩 Exécution avec logique custom")

            # Hook custom start
            try:
                on_mission_start(self)
            except Exception as e:
                self.get_logger().error(f"[custom] Erreur on_mission_start: {e}")

            # Boucle custom
            while rclpy.ok():
                self.state = MissionState.CUSTOM 
                # Annulation
                if goal_handle.is_cancel_requested:
                    result = DoMission.Result()
                    result.result_code = 1
                    result.result_message = "Mission annulée"
                    goal_handle.canceled()
                    return result

                elapsed = int(time.time() - t0)

                # Hook step
                try:
                    end_msg = on_mission_step(self, elapsed)
                    if end_msg is not None:
                        # Fin personnalisée
                        try:
                            on_mission_end(self)
                        except:
                            pass

                        result = DoMission.Result()
                        result.result_code = 0
                        result.result_message = end_msg
                        goal_handle.succeed()
                        return result

                except Exception as e:
                    self.get_logger().error(f"[custom] Erreur on_mission_step: {e}")

                time.sleep(1)

        # ===============================
        #     MODE PAR DÉFAUT
        # ===============================
        else:
            self.get_logger().info("🧱 Exécution en mode par défaut")
            self.start_exploration()

            while rclpy.ok():
                # Annulation ?
                if goal_handle.is_cancel_requested:
                    self.get_logger().info("🟥 Mission annulée (default)")
                    self.stop_exploration()
                    self.state = MissionState.WAIT
                    result = DoMission.Result()
                    result.result_code = 1
                    result.result_message = "Mission annulée"
                    goal_handle.canceled()
                    return result

                # Feedback standard
                fb = DoMission.Feedback()
                fb.time_elapsed = int(time.time() - t0)
                fb.percent_complete = 0.0
                goal_handle.publish_feedback(fb)

                time.sleep(1.0)

        # Fin par défaut
        self.stop_exploration()
        self.state = MissionState.WAIT

        result = DoMission.Result()
        result.result_code = 0
        result.result_message = "Mission terminée avec succès"
        goal_handle.succeed()
        return result


# ==========================================================
def main(args=None):
    rclpy.init(args=args)
    node = MissionServer()
    executor = MultiThreadedExecutor()
    executor.add_node(node)

    try:
        rclpy.spin(node, executor=executor)
    except KeyboardInterrupt:
        node.get_logger().info("Mission Server interrompu")
        node.stop_exploration()
    finally:
        node.destroy_node()
        rclpy.shutdown()
