"""
===========================================================
 Fichier de mission simple à éditer
===========================================================

Ce fichier représente un exemple minimal de "logique de mission"
pour un robot. Il peut être édité via ton éditeur de code
pour que l'utilisateur modifie facilement le comportement
du robot entre deux missions.

SEULEMENT les fonctions ci-dessous devraient être modifiées.
Le MissionServer principal les appellera au bon moment.

-----------------------------------------------------------
 SECTIONS PRINCIPALES

 1.  on_mission_start()      → Appelé au tout début de la mission
 2.  on_mission_step()       → Appelé régulièrement pendant la mission
 3.  on_mission_cancel()     → Appelé lorsqu’une annulation est demandée
 4.  on_mission_end()        → Appelé à la fin d'une mission réussie
-----------------------------------------------------------

Chaque fonction possède déjà des exemples simples.
L’utilisateur peut les remplacer par son propre comportement :
- lancer / arrêter l'exploration
- publier des commandes
- attendre un événement
- changer la logique de feedback
- etc.

"""

import time


# ===========================================================
#                1.   DÉBUT DE LA MISSION
# ===========================================================
def on_mission_start(node):
    """
    Fonction appelée *une seule fois*, juste après que l’utilisateur
    démarre une mission. Utilisée pour préparer le robot.

    Paramètres
    ----------
    node : MissionServer
        Le nœud principal, permettant d’appeler :
        - node.start_exploration()
        - node.stop_exploration()
        - node.get_logger().info()
        - publier des topics, etc.
    """

    node.get_logger().info("🔧 [mission_logic] Initialisation de la mission...")
    node.start_exploration()   # Exemple : commencer exploration immédiatement
    time.sleep(0.5)

    # L'utilisateur peut ajouter ici :
    # - un déplacement initial
    # - une attente
    # - un reset de variables
    # - un envoi d'ordre spécifique


# ===========================================================
#                2.   BOUCLE PRINCIPALE (STEP)
# ===========================================================
def on_mission_step(node, elapsed_time):
    """
    Fonction appelée en continu pendant la mission (toutes les secondes
    dans l’exemple par défaut du MissionServer).

    Paramètres
    ----------
    node : MissionServer
    elapsed_time : int
        Temps écoulé depuis le début de la mission (en secondes)

    Retour attendu
    --------------
    - None → continuer la mission
    - Un string → terminer la mission avec un message (ex: "objectif atteint")
    """

    # Exemple : arrêter la mission après 20 secondes
    if elapsed_time >= 20:
        return "Mission complétée automatiquement après 20s"

    # Exemple simple : affichage régulier
    if elapsed_time % 5 == 0:
        node.get_logger().info(f"⏳ Mission en cours ({elapsed_time}s)")

    # Pas de fin prématurée → continuer
    return None


# ===========================================================
#                3.   ANNULATION DE MISSION
# ===========================================================
def on_mission_cancel(node):
    """
    Fonction appelée lorsque l'utilisateur annule la mission (action cancel).

    Paramètres
    ----------
    node : MissionServer

    Cette fonction permet d'arrêter proprement le robot :
    - stopper l'exploration
    - couper la navigation
    - libérer des ressources
    """

    node.get_logger().info("🟥 [mission_logic] Annulation reçue → arrêt robot")
    node.stop_exploration()
    time.sleep(0.3)


# ===========================================================
#                4.   FIN NORMALE DE LA MISSION
# ===========================================================
def on_mission_end(node):
    """
    Fonction appelée lorsque la mission se termine *normalement*
    (pas annulée, pas d'erreur).

    Paramètres
    ----------
    node : MissionServer

    Utile pour :
    - arrêter exploration
    - recentrer le robot
    - envoyer un message de fin
    """

    node.get_logger().info("🏁 [mission_logic] Mission terminée proprement")
    node.stop_exploration()
    time.sleep(0.2)
