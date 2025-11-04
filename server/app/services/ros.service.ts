process.env.ROS_DOMAIN_ID = '66';
import { Injectable, Logger, OnModuleInit } from '@nestjs/common';
import * as rclnodejs from 'rclnodejs';
import { LimoObject } from '@app/interfaces/LimoObject';
import { map } from 'rxjs';
import { SocketService } from './socket/socket.service';

@Injectable()
export class RosService implements OnModuleInit {
  private readonly logger = new Logger(RosService.name);
  private limoList: LimoObject[];
  private mappingNode: rclnodejs.Node | undefined;
  private mappingSubscription: rclnodejs.Subscription | undefined;

  constructor(private socketService: SocketService) {}

  async onModuleInit() {
    await rclnodejs.init();
    const nodeLimo1 = new rclnodejs.Node('identify_client_backend', 'limo1');
    const nodeLimo2 = new rclnodejs.Node('identify_client_backend', 'limo2');
    const Trigger = (rclnodejs.require('std_srvs') as any).srv.Trigger;
    const clientLimo1 = nodeLimo1.createClient(Trigger, 'identify_robot');
    const clientLimo2 = nodeLimo2.createClient(Trigger, 'identify_robot');
    this.limoList = [
      { node: nodeLimo1, identifyClient: clientLimo1 },
      { node: nodeLimo2, identifyClient: clientLimo2 }
    ];
    nodeLimo1.spin();
    nodeLimo2.spin();
    this.setupMappingListner();
    this.logger.log('ROS2 client prêt !');
  }

  private setupMappingListner() {
    this.mappingNode = new rclnodejs.Node('mapping_listener_backend');
    this.mappingSubscription = this.mappingNode.createSubscription(
      'nav_msgs/msg/OccupancyGrid',
      '/limo1/map',
      (msg) => {
        console.log('Received map data:', msg);
        this.socketService.sendMapToAllSockets(msg);
      }
    );
    this.mappingNode.spin();
    console.log('ROS2 subscriber started');
  }

  async identifyRobot(id: number): Promise<{ success: boolean; message: string }> {
    const obj = this.limoList[id - 1];
    if (!obj.identifyClient) {
      this.logger.error('Client ROS2 non initialisé');
      return { success: false, message: 'ROS2 non initialisé' };
    }

    return new Promise((resolve) => {
      const request = new ((rclnodejs.require('std_srvs') as any).srv.Trigger.Request)();
      obj.identifyClient.sendRequest(request, (response) => {
        if (response) resolve({ success: response.success, message: response.message });
        else resolve({ success: false, message: 'Échec de l’appel ROS2' });
      });
    });
  }


}