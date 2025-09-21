const express = require('express');
const rclnodejs = require('rclnodejs');

const app = express();
const port = 3000;
const cors = require('cors');
app.use(cors());

let client; // client ROS2 global

async function initRos() {
    await rclnodejs.init();
    const node = rclnodejs.createNode('identify_client_backend');
    const Trigger = rclnodejs.require('std_srvs').srv.Trigger;

    client = node.createClient(Trigger, 'identify_robot');

    rclnodejs.spin(node);
    console.log("ROS2 client prêt !");
}

app.get('/identify', async (req, res) => {
    console.log("Requête HTTP reçue : /identify");

    if (!client) {
        console.error("Client ROS2 non initialisé");
        return res.status(500).json({ success: false, message: 'ROS2 non initialisé' });
    }

    const request = new (rclnodejs.require('std_srvs').srv.Trigger.Request)();
    console.log("Envoi de la requête ROS2...");

    client.sendRequest(request, (response) => {
        console.log("Réponse ROS2 :", response);

        if (response) {
            return res.json({
                success: response.success,
                message: response.message,
            });
        } else {
            return res.status(500).json({ success: false, message: 'Échec de l’appel ROS2' });
        }
    });
});

app.listen(port, '0.0.0.0', async () => {
    console.log(` Backend lancé sur http://0.0.0.0:${port}`);
    await initRos();
});