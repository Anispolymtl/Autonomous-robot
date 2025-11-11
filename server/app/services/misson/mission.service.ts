import { Injectable, OnModuleInit } from "@nestjs/common";
import * as rclnodejs from 'rclnodejs';

@Injectable()
export class MissionService implements OnModuleInit {
    private missionNodes: Record<string, rclnodejs.Node> = {};
    private missionClients: Record<string, any> = {};
    private goalHandles: Record<string, any> = {};
    private DoMission: any;

    /** Robots disponibles — adapter selon ton contexte */
    private robots: string[] = ['limo1']; // ⚙️ mets ['limo1', 'limo2'] pour la simulation

    async onModuleInit() {
        this.DoMission = (rclnodejs.require('limo_interfaces') as any).action.DoMission;

        for (const ns of this.robots) {
            const node = new rclnodejs.Node(`mission_client_${ns}`, ns);
            const client = new rclnodejs.ActionClient(node, 'limo_interfaces/action/DoMission', 'do_mission');
            node.spin();

            this.missionNodes[ns] = node;
            this.missionClients[ns] = client;

            console.log(`Mission client prêt pour ${ns}`);
        }
    }

    async startMission(): Promise<void> {
        for (const ns of this.robots) {
            const client = this.missionClients[ns];
            if (!client) continue;

            console.log(`⏳ Attente du serveur d'action pour ${ns}...`);
            const ready = await client.waitForServer(5000);
            if (!ready) {
                console.error(`Serveur d'action non disponible pour ${ns}`);
                continue;
            }

            const goal = new this.DoMission.Goal();
            goal.mission_length = BigInt(0);
            const goalHandle = await client.sendGoal(goal);

            if (!goalHandle.accepted) {
                console.error(`Goal rejeté par le serveur ${ns}`);
                continue;
            }

            this.goalHandles[ns] = goalHandle;
            console.log(`Mission démarrée pour ${ns}`);
        }
    }

    async stopMission(): Promise<void> {
        for (const ns of Object.keys(this.goalHandles)) {
            const handle = this.goalHandles[ns];
            if (handle) {
                await handle.cancelGoal();
                console.log(`Mission annulée pour ${ns}`);
                delete this.goalHandles[ns];
            }
        }
    }
}
