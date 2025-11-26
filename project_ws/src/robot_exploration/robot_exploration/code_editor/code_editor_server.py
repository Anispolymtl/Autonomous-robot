# code_editor_server.py
import rclpy
from rclpy.node import Node
from limo_interfaces.srv import GetScript, SaveScript
import pathlib
import subprocess
import threading
import time


# 1) Fichier à modifier : dans le même dossier que ce script
SCRIPT_PATH = pathlib.Path(__file__).resolve().parent / "custom_mission.py"


class CodeEditorNode(Node):
    def __init__(self):
        super().__init__("code_editor")

        self.get_logger().info(f"[CodeEditor] Editing file: {SCRIPT_PATH}")

        self.get_service = self.create_service(
            GetScript,
            "get_robot_script",
            self.get_script_callback
        )

        self.save_service = self.create_service(
            SaveScript,
            "save_robot_script",
            self.save_script_callback
        )


    def restart_mission_servers(self):
        """
        Kill and restart mission_server nodes for limo1 and limo2
        so they reload the updated custom_mission.py.
        """

        self.get_logger().info("🟥 Killing mission_server processes...")

        # Arrête tous les mission_server
        subprocess.run(["pkill", "-f", "mission_server"], check=False)
        time.sleep(0.4)

        self.get_logger().info("🔪 mission_server killed. Restarting...")

        # Command template
        def launch(namespace):
            subprocess.Popen([
                "ros2", "run", "robot_exploration", "mission_server",
                "--ros-args",
                "-p", "use_custom_logic:=true",
                "-r", f"__ns:=/{namespace}"
            ])

        # Redémarre limo1 + limo2
        threading.Thread(target=launch, args=("limo1",), daemon=True).start()
        threading.Thread(target=launch, args=("limo2",), daemon=True).start()

        self.get_logger().info("🚀 mission_server restarted for limo1 and limo2.")


    def get_script_callback(self, request, response):
        try:
            with open(SCRIPT_PATH, "r") as f:
                response.code = f.read()
            response.success = True
            response.message = "Script loaded"
        except Exception as e:
            response.success = False
            response.message = f"Error reading script: {e}"

        return response

    def save_script_callback(self, request, response):
        new_code = request.new_code

        self.get_logger().info(f"Sauvegarde en cours")

        # Vérification syntaxique avant sauvegarde
        try:
            compile(new_code, str(SCRIPT_PATH), 'exec')
        except Exception as e:
            self.get_logger().info(f"Echec de la sauvegarde")
            response.success = False
            response.message = f"❌ Syntax error: {str(e)}"
            return response

        # Sauvegarde
        try:
            with open(SCRIPT_PATH, "w") as f:
                f.write(new_code)

            self.get_logger().info(f"Sauvegarde reussi")
            response.success = True
            response.message = "✅ Script saved successfully"
        except Exception as e:
            response.success = False
            response.message = f"Error writing file: {e}"

        if response.success:
            self.get_logger().info("🔄 Restarting MissionServers...")
            self.restart_mission_servers()

        return response


def main(args=None):
    rclpy.init(args=args)
    node = CodeEditorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
