#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.executors import MultiThreadedExecutor
from limo_interfaces.action import DoMission
from limo_interfaces.msg import RobotState as RobotStateMsg
from limo_interfaces.srv import SetRobotState
from std_msgs.msg import Bool
from threading import Lock
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
#                Mission Server principal
# ==========================================================
class MissionServer(Node):
    def __init__(self):
        super().__init__("mission_server")

        # Paramètre ROS2
        self.declare_parameter("use_custom_logic", False)
        self.use_custom_logic = self.get_parameter("use_custom_logic").get_parameter_value().bool_value

        # Variables d'état
        self.current_state = None
        self._state_lock = Lock()
        self._mission_active = False

        # Client State Manager
        self.state_client = self.create_client(SetRobotState, 'set_state')
        self.get_logger().info("⏳ Attente du State Manager...")
        while not self.state_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("State Manager non disponible, nouvelle tentative...")
        self.get_logger().info("✅ State Manager connecté")

        # Subscriber pour écouter les changements d'état
        self.state_sub = self.create_subscription(
            RobotStateMsg,
            'robot_state',
            self._on_state_changed,
            10
        )

        # Publisher pour contrôler l'exploration
        self.resume_pub = self.create_publisher(Bool, "explore/resume", 10)

        # Action Server
        self._action_server = ActionServer(
            self,
            DoMission,
            "do_mission",
            execute_callback=self._execute_mission,
            goal_callback=self._on_goal_received,
            cancel_callback=self._on_cancel_requested,
        )

        # État initial
        initial_state = RobotStateMsg.CUSTOM_MISSION if self.use_custom_logic else RobotStateMsg.WAIT
        self._set_state(initial_state)

    # ==========================================================
    #                  GESTION D'ÉTAT
    # ==========================================================

    def _set_state(self, state: int):
        """Change l'état via le State Manager"""
        request = SetRobotState.Request()
        request.state = state
        self.state_client.call_async(request)

    def _on_state_changed(self, msg):
        """
        Callback pour les changements d'état.
        Gère automatiquement l'exploration selon l'état.
        """
        with self._state_lock:
            old_state = self.current_state
            self.current_state = msg.state

            # Seulement si une mission est active
            if not self._mission_active:
                return

            # Mode par défaut : gérer l'exploration
            if not self.use_custom_logic:
                # Arrêt de l'exploration
                if old_state == RobotStateMsg.EXPLORATION and self.current_state != RobotStateMsg.EXPLORATION:
                    self.get_logger().info("⛔ Arrêt de l'exploration")
                    self.resume_pub.publish(Bool(data=False))

                # Démarrage/Reprise de l'exploration
                elif self.current_state == RobotStateMsg.EXPLORATION and old_state != RobotStateMsg.EXPLORATION:
                    self.get_logger().info("🟢 (Re)démarrage de l'exploration")
                    self.resume_pub.publish(Bool(data=True))

    # ==========================================================
    #                  ACTION SERVER
    # ==========================================================

    def _on_goal_received(self, goal_request):
        """Accepte ou rejette un goal"""
        self.get_logger().info("🎯 Goal reçu")
        return GoalResponse.ACCEPT

    def _on_cancel_requested(self, goal_handle):
        """Gère l'annulation d'une mission"""
        self.get_logger().info("🟥 Annulation demandée")

        # Arrêter l'exploration si nécessaire
        with self._state_lock:
            if self.current_state == RobotStateMsg.EXPLORATION:
                self.resume_pub.publish(Bool(data=False))

        # Hook custom
        if self.use_custom_logic and CUSTOM_LOGIC_AVAILABLE:
            try:
                on_mission_cancel(self)
            except Exception as e:
                self.get_logger().error(f"[custom] Erreur on_mission_cancel: {e}")

        # Retour à WAIT
        self._set_state(RobotStateMsg.WAIT)

        return CancelResponse.ACCEPT

    # ==========================================================
    #                  EXÉCUTION MISSIONS
    # ==========================================================

    async def _execute_mission(self, goal_handle):
        """Point d'entrée principal pour l'exécution"""
        self._mission_active = True

        try:
            if self.use_custom_logic and CUSTOM_LOGIC_AVAILABLE:
                result = await self._run_custom_mission(goal_handle)
            else:
                result = await self._run_default_mission(goal_handle)

            return result

        finally:
            self._mission_active = False

    # ----------------------------------------------------------
    #                  MISSION PAR DÉFAUT
    # ----------------------------------------------------------

    async def _run_default_mission(self, goal_handle):
        """
        Mission par défaut : exploration continue
        """
        self.get_logger().info("🚀 Mission START (mode par défaut)")
        start_time = time.time()

        # Démarrer l'exploration
        self._set_state(RobotStateMsg.EXPLORATION)

        # Boucle principale
        while rclpy.ok():
            # Vérifier annulation
            if goal_handle.is_cancel_requested:
                self.get_logger().info("🟥 Mission annulée")
                with self._state_lock:
                    if self.current_state == RobotStateMsg.EXPLORATION:
                        self.resume_pub.publish(Bool(data=False))
                self._set_state(RobotStateMsg.WAIT)

                result = DoMission.Result()
                result.result_code = 1
                result.result_message = "Mission annulée"
                goal_handle.canceled()
                return result

            # Publier feedback
            elapsed = int(time.time() - start_time)
            feedback = DoMission.Feedback()
            feedback.time_elapsed = elapsed
            feedback.percent_complete = 0.0
            goal_handle.publish_feedback(feedback)

            time.sleep(1.0)

        # Fin (ne devrait jamais arriver)
        with self._state_lock:
            if self.current_state == RobotStateMsg.EXPLORATION:
                self.resume_pub.publish(Bool(data=False))
        self._set_state(RobotStateMsg.WAIT)

        result = DoMission.Result()
        result.result_code = 0
        result.result_message = "Mission terminée"
        goal_handle.succeed()
        return result

    # ----------------------------------------------------------
    #                  MISSION CUSTOM
    # ----------------------------------------------------------

    async def _run_custom_mission(self, goal_handle):
        """
        Mission personnalisée : utilise les hooks custom
        """
        self.get_logger().info("🚀 Mission START (mode custom)")
        start_time = time.time()

        # État custom
        self._set_state(RobotStateMsg.CUSTOM_MISSION)

        # Hook: démarrage
        try:
            on_mission_start(self)
        except Exception as e:
            self.get_logger().error(f"[custom] Erreur on_mission_start: {e}")

        # Boucle principale
        while rclpy.ok():
            # Vérifier annulation
            if goal_handle.is_cancel_requested:
                self.get_logger().info("🟥 Mission annulée")

                # Hook: annulation
                try:
                    on_mission_cancel(self)
                except Exception as e:
                    self.get_logger().error(f"[custom] Erreur on_mission_cancel: {e}")

                self._set_state(RobotStateMsg.WAIT)

                result = DoMission.Result()
                result.result_code = 1
                result.result_message = "Mission annulée"
                goal_handle.canceled()
                return result

            elapsed = int(time.time() - start_time)

            # Hook: step
            try:
                end_msg = on_mission_step(self, elapsed)

                if end_msg is not None:
                    # Mission terminée
                    self.get_logger().info("✅ Mission terminée")

                    # Hook: fin
                    try:
                        on_mission_end(self)
                    except Exception as e:
                        self.get_logger().warn(f"[custom] Erreur on_mission_end: {e}")

                    self._set_state(RobotStateMsg.WAIT)

                    result = DoMission.Result()
                    result.result_code = 0
                    result.result_message = end_msg
                    goal_handle.succeed()
                    return result

            except Exception as e:
                self.get_logger().error(f"[custom] Erreur on_mission_step: {e}")

            time.sleep(1.0)


# ==========================================================
def main(args=None):
    rclpy.init(args=args)
    node = MissionServer()
    executor = MultiThreadedExecutor()
    executor.add_node(node)

    try:
        rclpy.spin(node, executor=executor)
    except KeyboardInterrupt:
        node.get_logger().info("🛑 Mission Server interrompu")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()