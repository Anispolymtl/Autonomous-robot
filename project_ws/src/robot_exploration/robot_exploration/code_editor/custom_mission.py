"""
===========================================================
      Template de mission personnalisée (custom mission)
===========================================================

Ce fichier permet de définir une logique de mission personnalisée
pour le MissionServer lorsqu'il est lancé avec :

    use_custom_logic := true

Le comportement est contrôlé par 4 fonctions principales :
-----------------------------------------------------------
 1. on_mission_start(node)        → Appelée au début
 2. on_mission_step(node, elapsed) → Appelée chaque seconde
 3. on_mission_end(node)           → Appelée en fin normale
 4. on_mission_cancel(node)        → Appelée si annulée
-----------------------------------------------------------

IMPORTANT:
- on_mission_step() doit retourner None pour continuer
- on_mission_step() retourne une string pour terminer
- Utilisez node.get_logger().info() pour les logs
- Accédez aux services/topics via node.create_*()

"""

from geometry_msgs.msg import Twist


# ===========================================================
#               VARIABLES GLOBALES (optionnel)
# ===========================================================

# Publishers initialisés dans on_mission_start
_cmd_pub = None

# Variables de mission
_mission_data = {}


# ===========================================================
#            1. DÉBUT DE LA MISSION
# ===========================================================

def on_mission_start(node):
    """
    Appelée UNE SEULE FOIS au démarrage de la mission.
    
    Utilisez cette fonction pour :
    - Créer des publishers/subscribers
    - Initialiser des variables
    - Configurer l'état initial
    
    NOTE: L'état du robot reste en CUSTOM_MISSION durant toute la mission.
    
    Args:
        node: Instance du MissionServer
    """
    global _cmd_pub, _mission_data
    
    # Créer le publisher pour contrôler le robot
    _cmd_pub = node.create_publisher(Twist, "cmd_vel", 10)
    
    # Initialiser les données de mission
    _mission_data = {
        'phase': 0,
        'counter': 0,
        'max_duration': 60  # Durée max en secondes
    }
    
    node.get_logger().info("🚀 [Custom] Mission démarrée")
    
    # Exemple : Arrêter le robot au départ
    _stop_robot()


# ===========================================================
#            2. BOUCLE PRINCIPALE (STEP)
# ===========================================================

def on_mission_step(node, elapsed_time):
    """
    Appelée RÉGULIÈREMENT (environ 1 Hz) pendant la mission.
    
    C'est ici que vous implémentez votre logique principale.
    
    Args:
        node: Instance du MissionServer
        elapsed_time: Temps écoulé depuis le début (secondes)
    
    Returns:
        None: Continue la mission
        str: Termine la mission avec ce message
    """
    global _mission_data
    
    # Exemple : Terminer après une durée max
    if elapsed_time >= _mission_data['max_duration']:
        return f"Mission terminée après {elapsed_time}s"
    
    # Exemple : Alterner entre avancer et reculer
    cycle_duration = 5  # secondes
    phase = int(elapsed_time / cycle_duration) % 2
    
    if phase != _mission_data['phase']:
        _mission_data['phase'] = phase
        _mission_data['counter'] += 1
        
        if phase == 0:
            node.get_logger().info(f"➡️ [Custom] Phase AVANT (cycle {_mission_data['counter']})")
            _move_forward(speed=0.2)
        else:
            node.get_logger().info(f"⬅️ [Custom] Phase ARRIÈRE (cycle {_mission_data['counter']})")
            _move_backward(speed=0.2)
    
    # Continuer la mission
    return None


# ===========================================================
#            3. FIN NORMALE DE LA MISSION
# ===========================================================

def on_mission_end(node):
    """
    Appelée lorsque la mission se termine NORMALEMENT.
    (on_mission_step a retourné une string)
    
    Utilisez cette fonction pour :
    - Arrêter proprement le robot
    - Nettoyer les ressources
    - Enregistrer des données
    
    Args:
        node: Instance du MissionServer
    """
    node.get_logger().info("🏁 [Custom] Mission terminée normalement")
    
    # Arrêter le robot
    _stop_robot()
    
    # Nettoyer les variables globales
    global _mission_data
    _mission_data.clear()


# ===========================================================
#            4. ANNULATION DE LA MISSION
# ===========================================================

def on_mission_cancel(node):
    """
    Appelée lorsque la mission est ANNULÉE par l'utilisateur.
    
    Utilisez cette fonction pour :
    - Arrêter d'urgence le robot
    - Nettoyer les ressources
    - Sauvegarder l'état si nécessaire
    
    Args:
        node: Instance du MissionServer
    """
    node.get_logger().info("🟥 [Custom] Mission annulée")
    
    # Arrêter immédiatement le robot
    _stop_robot()
    
    # Nettoyer
    global _mission_data
    _mission_data.clear()


# ===========================================================
#               FONCTIONS UTILITAIRES
# ===========================================================

def _stop_robot():
    """Arrête complètement le robot"""
    global _cmd_pub
    if _cmd_pub:
        msg = Twist()  # Toutes les vitesses à 0
        _cmd_pub.publish(msg)


def _move_forward(speed=0.2):
    """Fait avancer le robot"""
    global _cmd_pub
    if _cmd_pub:
        msg = Twist()
        msg.linear.x = speed
        _cmd_pub.publish(msg)


def _move_backward(speed=0.2):
    """Fait reculer le robot"""
    global _cmd_pub
    if _cmd_pub:
        msg = Twist()
        msg.linear.x = -speed
        _cmd_pub.publish(msg)


def _turn_left(angular_speed=0.5):
    """Fait tourner le robot à gauche"""
    global _cmd_pub
    if _cmd_pub:
        msg = Twist()
        msg.angular.z = angular_speed
        _cmd_pub.publish(msg)


def _turn_right(angular_speed=0.5):
    """Fait tourner le robot à droite"""
    global _cmd_pub
    if _cmd_pub:
        msg = Twist()
        msg.angular.z = -angular_speed
        _cmd_pub.publish(msg)


# ===========================================================
#               EXEMPLES D'UTILISATION
# ===========================================================

"""
EXEMPLE 1 : Mission simple avec timeout
----------------------------------------
def on_mission_step(node, elapsed_time):
    if elapsed_time >= 30:
        return "Timeout après 30s"
    
    _move_forward(speed=0.3)
    return None


EXEMPLE 2 : Mission avec phases multiples
------------------------------------------
def on_mission_step(node, elapsed_time):
    if elapsed_time < 10:
        node.get_logger().info("Phase 1: Avance")
        _move_forward(0.3)
    elif elapsed_time < 20:
        node.get_logger().info("Phase 2: Tourne")
        _turn_left(0.5)
    elif elapsed_time < 30:
        node.get_logger().info("Phase 3: Recule")
        _move_backward(0.2)
    else:
        return "Mission en 3 phases terminée"
    
    return None


EXEMPLE 3 : Mission avec compteur de cycles
--------------------------------------------
_cycle_count = 0

def on_mission_step(node, elapsed_time):
    global _cycle_count
    
    phase = int(elapsed_time / 5) % 2
    
    if phase == 0 and elapsed_time > 0 and elapsed_time % 5 < 1:
        _cycle_count += 1
        node.get_logger().info(f"Cycle {_cycle_count}")
    
    if _cycle_count >= 10:
        return f"Mission terminée après {_cycle_count} cycles"
    
    if phase == 0:
        _move_forward(0.2)
    else:
        _turn_right(0.3)
    
    return None


EXEMPLE 4 : Patrouille en carré
--------------------------------
def on_mission_step(node, elapsed_time):
    # Chaque côté dure 5 secondes
    side_duration = 5
    total_sides = 4
    
    side_number = int(elapsed_time / side_duration) % (total_sides + 1)
    
    if side_number < total_sides:
        time_in_side = elapsed_time % side_duration
        
        if time_in_side < 3:
            # Avancer pendant 3 secondes
            _move_forward(0.2)
        else:
            # Tourner pendant 2 secondes
            _turn_left(0.5)
    else:
        return "Patrouille carrée terminée"
    
    return None
"""