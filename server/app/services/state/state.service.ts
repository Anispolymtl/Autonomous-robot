import { Injectable } from "@nestjs/common";
import * as rclnodejs from 'rclnodejs';
import { SocketService } from '@app/services/socket/socket.service';

type RobotId = 'limo1' | 'limo2';

@Injectable()
export class StateService {
    private readonly robotIds: RobotId[] = ['limo1', 'limo2'];
    private readonly robotStateConstants: any;
    private readonly stateLabels: Record<number, string>;

    private stateListeners: Partial<Record<RobotId, rclnodejs.Node>> = {};
    private stateSubscriptions: Partial<Record<RobotId, rclnodejs.Subscription>> = {};

    constructor(private socketService: SocketService) {
        this.robotStateConstants = (rclnodejs.require('limo_interfaces') as any).msg.RobotState;
        this.stateLabels = {
            [this.robotStateConstants.WAIT]: 'En attente',
            [this.robotStateConstants.EXPLORATION]: 'Exploration',
            [this.robotStateConstants.NAVIGATION]: 'Navigation',
            [this.robotStateConstants.RETURN_TO_BASE]: 'Retour a la base',
            [this.robotStateConstants.CUSTOM_MISSION]: 'Mission personnalisee',
        };
    }

    initStateService() {
        this.robotIds.forEach((robotId) => this.setupStateListener(robotId));
        console.log('ROS2 subscriber started');
    }

    private setupStateListener(robotId: RobotId) {
        const node = new rclnodejs.Node(`state_listener_backend_${robotId}`, robotId);
        const topic = `/${robotId}/robot_state`;

        const subscription = node.createSubscription(
            this.robotStateConstants,
            topic,
            (msg: { state?: number }) => {
                const stateValue = msg?.state;
                const formattedState = stateValue !== undefined
                    ? this.stateLabels[stateValue] ?? `Inconnu (${stateValue})`
                    : 'Inconnu';
                this.socketService.sendStateToAllSockets(robotId, formattedState);
            }
        );

        node.spin();
        this.stateListeners[robotId] = node;
        this.stateSubscriptions[robotId] = subscription;
    }
}
