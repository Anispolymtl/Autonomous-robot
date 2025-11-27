"""
===========================================================
      Exemple de mission personnalisée (custom mission)
===========================================================

Ce fichier contient une logique de mission entièrement
personnalisée, utilisée lorsque le MissionServer est lancé
avec le paramètre :

    use_custom_logic := true

L’objectif de cet exemple est de montrer comment réaliser
une mission simple : le robot alterne entre avancer et
reculer toutes les 2 secondes.

Le comportement est ainsi contrôlé par 4 fonctions :
-----------------------------------------------------------
 1. on_mission_start()  → appelée au début de la mission
 2. on_mission_step()   → appelée régulièrement (1 Hz)
 3. on_mission_cancel() → appelée lors d'une annulation
 4. on_mission_end()    → appelée à la fin normale
-----------------------------------------------------------

L’utilisateur peut modifier :
- les vitesses
- la durée des cycles
- la logique d’alternance
- la publication des commandes
- les actions à la fin/annulation

"""

import time
from geometry_msgs.msg import Twist


# ============================================================
#                 PARAMÈTRES GÉNÉRAUX DU COMPORTEMENT
# ============================================================

# Vitesse linéaire avant (m/s)
FORWARD_SPEED = 0.20

# Vitesse linéaire arrière (m/s)
BACKWARD_SPEED = -0.20

# Durée d’un cycle avant → arrière (en secondes)
CYCLE_DURATION = 2.0

# Publisher (initialisé dans on_mission_start)
cmd_pub = None



# ============================================================
#                1.   DÉBUT DE LA MISSION
# ============================================================
def on_mission_start(node):
    """
    Appelée une seule fois lors du démarrage de la mission.

    Le rôle principal est :
    - initialiser les publishers nécessaires
    - préparer les variables
    - envoyer un message de démarrage
    """

    global cmd_pub
    cmd_pub = node.create_publisher(Twist, "cmd_vel", 10)

    node.get_logger().info(
        "🚀 [custom_mission] Mission personnalisée démarrée : "
        "le robot va alterner avancer/reculer toutes les 2 sec."
    )



# ============================================================
#                2.   BOUCLE PRINCIPALE (STEP)
# ============================================================
def on_mission_step(node, elapsed_time):
    """
    Appelée en continu (environ toutes les secondes) pendant la mission.

    Paramètres
    ----------
    node : MissionServer
        Le nœud principal.
    elapsed_time : int
        Temps écoulé depuis le début (en secondes)

    Retour attendu
    --------------
    - None → continuer la mission
    - str  → terminer la mission immédiatement avec un message
    """

    global cmd_pub
    msg = Twist()

    # Phase du cycle : 0 = avance, 1 = recule
    cycle_phase = int(elapsed_time / CYCLE_DURATION) % 2

    if cycle_phase == 0:
        msg.linear.x = FORWARD_SPEED
        node.get_logger().info("➡️ Avance")
    else:
        msg.linear.x = BACKWARD_SPEED
        node.get_logger().info("⬅️ Recule")

    # Publication de la commande sur /cmd_vel
    cmd_pub.publish(msg)

    # Pas de fin automatique → la mission continue
    return None



# ============================================================
#                3.   ANNULATION DE LA MISSION
# ============================================================
def on_mission_cancel(node):
    """
    Appelée lorsque l’utilisateur demande une annulation.

    Permet d'arrêter proprement le robot.
    """

    stop = Twist()  # msg vide = arrêt moteur
    node.create_publisher(Twist, "cmd_vel", 10).publish(stop)

    node.get_logger().info(
        "🟥 [custom_mission] Mission annulée : robot immobilisé."
    )

    time.sleep(0.2)



# ============================================================
#                4.   FIN NORMALE DE LA MISSION
# ============================================================
def on_mission_end(node):
    """
    Appelée uniquement lorsque la mission termine normalement
    (pas une annulation).

    Utile pour :
    - arrêter définitivement le robot
    - envoyer un message de fin
    - réaliser une action finale
    """

    stop = Twist()
    node.create_publisher(Twist, "cmd_vel", 10).publish(stop)

    node.get_logger().info(
        "🏁 [custom_mission] Mission terminée : robot arrêté proprement."
    )

    time.sleep(0.2)
