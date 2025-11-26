import time
from geometry_msgs.msg import Twist

# Vitesse linéaire avant/arrière
FORWARD_SPEED = 0.2
BACKWARD_SPEED = -0.2
CYCLE_DURATION = 2.0   # secondes avant → 2 sec arrière

cmd_pub = None


def on_mission_start(node):
    global cmd_pub
    cmd_pub = node.create_publisher(Twist, "cmd_vel", 10)
    node.get_logger().info("🚀 Mission démarrée : le robot va avancer/reculer en continu")


def on_mission_step(node, elapsed_time):
    global cmd_pub

    msg = Twist()

    # On alterne entre avant et arrière toutes les CYCLE_DURATION secondes
    cycle_phase = int(elapsed_time / CYCLE_DURATION) % 2

    if cycle_phase == 0:
        msg.linear.x = FORWARD_SPEED
        node.get_logger().info("➡️ Avance")
    else:
        msg.linear.x = BACKWARD_SPEED
        node.get_logger().info("⬅️ Recule")

    cmd_pub.publish(msg)

    # Pas de fin → None
    return None


def on_mission_cancel(node):
    stop = Twist()
    node.create_publisher(Twist, "cmd_vel", 10).publish(stop)
    node.get_logger().info("🟥 Mission annulée : arrêt du robot")


def on_mission_end(node):
    stop = Twist()
    node.create_publisher(Twist, "cmd_vel", 10).publish(stop)
    node.get_logger().info("🏁 Mission terminée : arrêt du robot")
