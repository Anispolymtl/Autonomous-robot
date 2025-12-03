#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from threading import Lock

# Import des messages/services personnalisés
from limo_interfaces.msg import RobotState as RobotStateMsg
from limo_interfaces.srv import SetRobotState
from std_msgs.msg import Bool


class StateManager(Node):
    """
    Gestionnaire d'état centralisé pour le robot avec enum personnalisé.
    
    Services:
        - /set_state (SetRobotState): Change l'état du robot
    
    Topics publiés:
        - /robot_state (RobotState): État courant du robot (enum uint8)
    """
    
    # Mapping pour les logs (correspondance avec les valeurs de l'enum)
    STATE_NAMES = {
        RobotStateMsg.WAIT: "En Attente",
        RobotStateMsg.EXPLORATION: "Exploration",
        RobotStateMsg.NAVIGATION: "Navigation",
        RobotStateMsg.RETURN_TO_BASE: "Retour à la base",
        RobotStateMsg.CUSTOM_MISSION: "Mission personnalisée"
    }
    
    def __init__(self):
        super().__init__('state_manager')
        
        # État initial
        self._current_state = RobotStateMsg.WAIT
        self._state_lock = Lock()
        
        # Paramètres
        self.declare_parameter('initial_state', RobotStateMsg.WAIT)
        initial_state = self.get_parameter('initial_state').value
        
        # Valider l'état initial
        if self._is_valid_state(initial_state):
            self._current_state = initial_state
            self.get_logger().info(
                f"📍 État initial: {self.STATE_NAMES[self._current_state]}"
            )
        else:
            self.get_logger().warn(
                f"⚠️ État initial invalide {initial_state}, utilisation de WAIT"
            )
        
        # Publisher
        self.state_pub = self.create_publisher(
            RobotStateMsg, 
            'robot_state', 
            10
        )
        # Permet de stopper l'exploration lorsqu'on quitte l'état EXPLORATION
        self.resume_pub = self.create_publisher(
            Bool,
            'explore/resume',
            10
        )
        
        # Service pour changer d'état
        self.set_state_srv = self.create_service(
            SetRobotState,
            'set_state',
            self.set_state_callback
        )
        
        # Timer pour publier l'état périodiquement (1 Hz)
        self.state_timer = self.create_timer(1.0, self.publish_state)
        
        self.get_logger().info("✅ State Manager initialisé")
        self.get_logger().info("📡 Publication sur /robot_state à 1 Hz")
        self.get_logger().info("🔧 Service disponible: /set_state")
        self.get_logger().info("📋 États disponibles:")
        for state_val, state_name in self.STATE_NAMES.items():
            self.get_logger().info(f"   {state_val}: {state_name}")
    
    def _is_valid_state(self, state: int) -> bool:
        """Vérifie si l'état est valide"""
        return state in self.STATE_NAMES

    def _stop_processes_for_transition(self, current_state: int, next_state: int):
        """
        Stoppe proprement les processus de l'état courant avant la transition.
        À étendre si d'autres états nécessitent un arrêt explicite.
        """
        if current_state == RobotStateMsg.EXPLORATION and next_state != RobotStateMsg.EXPLORATION:
            self.get_logger().info(
                f"⛔ Arrêt de l'exploration (transition vers {self.STATE_NAMES.get(next_state, next_state)})"
            )
            self.resume_pub.publish(Bool(data=False))
    
    def set_state_callback(self, request, response):
        """
        Callback du service pour changer d'état.
        
        Args:
            request.state (uint8): Valeur de l'enum (0-4)
        
        Returns:
            response.success (bool): True si la transition a réussi
            response.message (str): Message de confirmation ou d'erreur
        """
        requested_state = request.state
        
        self.get_logger().info(
            f"🔄 Demande de changement d'état: {self.STATE_NAMES.get(requested_state, 'INCONNU')}"
        )
        
        # Vérifier que l'état demandé est valide
        if not self._is_valid_state(requested_state):
            error_msg = (
                f"État invalide {requested_state}. États valides: "
                f"{list(self.STATE_NAMES.keys())}"
            )
            self.get_logger().error(f"❌ {error_msg}")
            response.success = False
            response.message = error_msg
            return response
        
        # Changer l'état (thread-safe)
        with self._state_lock:
            old_state = self._current_state
            if requested_state == old_state:
                info_msg = f"État inchangé: déjà en {self.STATE_NAMES[old_state]}"
                self.get_logger().info(f"ℹ️ {info_msg}")
                response.success = True
                response.message = info_msg
                return response

            self._stop_processes_for_transition(old_state, requested_state)
            self._current_state = requested_state
        
        # Log
        success_msg = (
            f"Transition: {self.STATE_NAMES[old_state]} → "
            f"{self.STATE_NAMES[requested_state]}"
        )
        self.get_logger().info(f"✅ {success_msg}")
        
        response.success = True
        response.message = success_msg
        
        return response
    
    def publish_state(self):
        """Publie l'état courant sur le topic /robot_state"""
        msg = RobotStateMsg()
        
        with self._state_lock:
            msg.state = self._current_state
        
        self.state_pub.publish(msg)
    
    def get_current_state(self) -> int:
        """Retourne l'état courant (thread-safe)"""
        with self._state_lock:
            return self._current_state


def main(args=None):
    rclpy.init(args=args)
    
    node = StateManager()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("🛑 State Manager arrêté")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
