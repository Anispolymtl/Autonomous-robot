import { Injectable, OnModuleInit } from "@nestjs/common";
import * as rclnodejs from 'rclnodejs';

@Injectable()
export class MissionService implements OnModuleInit {
    private missionNode1: rclnodejs.Node | undefined;
    private missionNode2: rclnodejs.Node | undefined;
    private missionClient1: any;
    private missionClient2: any;
    private DoMission: any;
    private goalHandle1: any;
    private goalHandle2: any;

    async onModuleInit() {
        this.missionNode1 = new rclnodejs.Node('mission_client_backend', 'limo1');
        this.DoMission = (rclnodejs.require('limo_interfaces') as any).action.DoMission;
        this.missionClient1 = new rclnodejs.ActionClient(this.missionNode1, 'limo_interfaces/action/DoMission', 'do_mission');
        this.missionNode1.spin();
        this.missionNode2 = new rclnodejs.Node('mission_client_backend', 'limo2');
        this.missionClient2 = new rclnodejs.ActionClient(this.missionNode2, 'limo_interfaces/action/DoMission', 'do_mission');
        this.missionNode2.spin();
        console.log('ROS2 Mission client prêt !');
    }

    async startMission(): Promise<any> {
        if (!this.missionClient1 || !this.missionClient2) {
            console.error('Client ROS2 non initialisé');
            return;
        }

        if (!await this.missionClient1.waitForServer(1000) || !await this.missionClient2.waitForServer(1000)) {
            console.error('The action servers are not up')
            return;
        }

        if (this.goalHandle1 || this.goalHandle2) {
            console.warn('Une mission est déjà en cours');
            return;
        }

        const goal = new this.DoMission.Goal();
        goal.mission_length = BigInt(0); // 0 pour une mission infinie

        this.goalHandle1 = await this.missionClient1.sendGoal(goal);
        this.goalHandle2 = await this.missionClient2.sendGoal(goal);

        if (!this.goalHandle1.accepted && !this.goalHandle2.accepted) {
            console.error('Le but de la mission a été rejeté par le serveur');
            return;
        }

        console.log('But de la mission accepté, attente du résultat...');

        // const { result1, status1 } = await this.goalHandle1.getResult();
        // const { result2, status2 } = await this.goalHandle2.getResult();

        // console.log('Résultat de la mission reçu :', result1, result2);
        
        // return result1;
    }

    async stopMission(): Promise<any> {
        if (!this.missionClient1 || !this.missionClient2) {
            console.error('Client ROS2 non initialisé');
            return;
        }

        if (!this.goalHandle1 || !this.goalHandle2) {
            console.warn('Aucune mission en cours à annuler');
            return;
        }
        
        await this.goalHandle1.cancelGoal();
        await this.goalHandle2.cancelGoal();
        this.goalHandle1 = null;
        this.goalHandle2 = null;


        console.log('Tous les buts de la mission ont été annulés avec succès');
    }
}
