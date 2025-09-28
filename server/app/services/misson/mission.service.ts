import { Injectable, OnModuleInit } from "@nestjs/common";
import * as rclnodejs from 'rclnodejs';


@Injectable()
export class MissionService implements OnModuleInit {
    private missionNode: rclnodejs.Node | undefined;
    private missionClient: any;
    private DoMission: any;
    private goalHandle: any;

    async onModuleInit() {
        this.missionNode = new rclnodejs.Node('mission_client_backend');
        this.DoMission = (rclnodejs.require('limo_interfaces') as any).action.DoMission;  
        this.missionClient = new rclnodejs.ActionClient(this.missionNode, 'limo_interfaces/action/DoMission', '/do_mission');        
        this.missionNode.spin();
        console.log('ROS2 Mission client prêt !');
    }

    async startMission(): Promise<any> {
        if (!this.missionClient) {
            console.error('Client ROS2 non initialisé');
            return;
        }

        if (this.goalHandle) {
            console.warn('Une mission est déjà en cours');
            return;
        }

        const goal = new this.DoMission.Goal();
        goal.mission_length = BigInt(0); // 0 pour une mission infinie

        this.goalHandle = await this.missionClient.sendGoal(goal);

        if (!this.goalHandle.accepted) {
            console.error('Le but de la mission a été rejeté par le serveur');
            return;
        }

        console.log('But de la mission accepté, attente du résultat...');

        const { result, status } = await this.goalHandle.getResult();

        console.log('Résultat de la mission reçu :', result);
        
        return result;
    }

    async stopMission(): Promise<any> {
        if (!this.missionClient) {
            console.error('Client ROS2 non initialisé');
            return;
        }

        if (!this.goalHandle) {
            console.warn('Aucune mission en cours à annuler');
            return;
        }
        
        await this.goalHandle.cancel();


        console.log('Tous les buts de la mission ont été annulés avec succès');
    }
    
        

}