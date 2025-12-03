import { Injectable } from "@nestjs/common";
import * as rclnodejs from 'rclnodejs';
import { SocketService } from '@app/services/socket/socket.service';
import { NavService } from '@app/services/nav/nav.service';

type RobotId = 'limo1' | 'limo2';

@Injectable()
export class StateService {
    private readonly robotIds: RobotId[] = ['limo1', 'limo2'];
    private readonly robotStateConstants: any;
    private readonly stateLabels: Record<number, string>;

    private stateListeners: Partial<Record<RobotId, rclnodejs.Node>> = {};
    private stateSubscriptions: Partial<Record<RobotId, rclnodejs.Subscription>> = {};

    constructor(
        private socketService: SocketService,
        // private navService: NavService,
    ) {
        this.robotStateConstants = this.loadRobotStateConstants();
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
            'limo_interfaces/msg/RobotState',
            topic,
            (msg: { state?: number }) => {
                const stateValue = msg?.state;
                const formattedState = stateValue !== undefined
                    ? this.stateLabels[stateValue] ?? `Inconnu (${stateValue})`
                    : 'Inconnu';

                // const shouldIgnoreWaitingState =
                //     stateValue === this.robotStateConstants.WAIT &&
                //     this.navService?.isReturnInProgress(robotId);
                // if (shouldIgnoreWaitingState) {
                //     return;
                // }

                this.socketService.sendStateToAllSockets(robotId, formattedState);
            }
        );

        node.spin();
        this.stateListeners[robotId] = node;
        this.stateSubscriptions[robotId] = subscription;
    }

    private loadRobotStateConstants() {
        try {
            const pkg: any = (rclnodejs.require as any)?.('limo_interfaces');
            const constants = pkg?.msg?.RobotState;
            if (constants) return constants;
        } catch (err) {
            console.warn('[STATE] Impossible de charger limo_interfaces/RobotState, utilisation des valeurs par defaut', err);
        }
        return {
            WAIT: 0,
            EXPLORATION: 1,
            NAVIGATION: 2,
            RETURN_TO_BASE: 3,
            CUSTOM_MISSION: 4,
        };
    }
}
