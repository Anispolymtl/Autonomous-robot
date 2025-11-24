# code_editor_server.py
import rclpy
from rclpy.node import Node
from limo_interfaces.srv import GetScript, SaveScript
import os

SCRIPT_PATH = os.path.join(
    os.path.dirname(__file__),   # ← prend le dossier actuel
    '..',                        # remonte d’un niveau
    'mission',
    'mission_logic.py'
)

class CodeEditorNode(Node):
    def __init__(self):
        super().__init__("code_editor")

        self.get_service = self.create_service(GetScript, "get_robot_script", self.get_script_callback)
        self.save_service = self.create_service(SaveScript, "save_robot_script", self.save_script_callback)

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

        # 1) Vérification syntaxique avant sauvegarde
        try:
            compile(new_code, SCRIPT_PATH, 'exec')
        except Exception as e:
            response.success = False
            response.message = f"❌ Syntax error: {str(e)}"
            return response

        # 2) Sauvegarde
        try:
            with open(SCRIPT_PATH, "w") as f:
                f.write(new_code)
            response.success = True
            response.message = "✅ Script saved successfully"
        except Exception as e:
            response.success = False
            response.message = f"Error writing file: {e}"

        return response


def main(args=None):
    rclpy.init(args=args)
    node = CodeEditorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
