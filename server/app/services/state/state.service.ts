import { Injectable } from "@nestjs/common";
import * as rclnodejs from 'rclnodejs';
import { SocketService } from '@app/services/socket/socket.service';

@Injectable()
export class StateService {
    private stateListner1: rclnodejs.Node | undefined;
    private stateListner2: rclnodejs.Node | undefined;
    private stateSubscription1: rclnodejs.Subscription | undefined;
    private stateSubscription2: rclnodejs.Subscription | undefined;
    
    constructor(private socketService: SocketService){}
    
    initStateService() {
        this.setupStateListner()
    }
    private setupStateListner() {
        this.stateListner1 = new rclnodejs.Node('state_listener_backend', 'limo1');
        this.stateSubscription1 = this.stateListner1.createSubscription(
            'std_msgs/msg/String',
            '/limo1/mission_state',
            (msg) => {
                this.socketService.sendStateToAllSockets('limo1', msg.data);
            }
        );
        this.stateListner1.spin();
        
        this.stateListner2 = new rclnodejs.Node('state_listener_backend', 'limo2');
        this.stateSubscription2 = this.stateListner2.createSubscription(
          'std_msgs/msg/String',
          '/limo2/mission_state',
          (msg) => {
            this.socketService.sendStateToAllSockets('limo2', msg.data);
          }
        );
        this.stateListner2.spin()

        console.log('ROS2 subscriber started');
    }
}