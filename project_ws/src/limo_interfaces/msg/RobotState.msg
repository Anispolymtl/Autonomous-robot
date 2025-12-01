# RobotState.msg

# États possibles du robot (enum-like constants)
uint8 WAIT = 0
uint8 EXPLORATION = 1
uint8 NAVIGATION = 2
uint8 RETURN_TO_BASE = 3
uint8 CUSTOM_MISSION = 4

# État courant
uint8 state