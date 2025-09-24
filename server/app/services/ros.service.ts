import { Injectable, Logger, OnModuleInit } from '@nestjs/common';
import * as rclnodejs from 'rclnodejs';

@Injectable()
export class RosService implements OnModuleInit {
  private readonly logger = new Logger(RosService.name);
  private client: any;

  async onModuleInit() {
    await rclnodejs.init();
    const node = new rclnodejs.Node('identify_client_backend');
    const Trigger = (rclnodejs.require('std_srvs') as any).srv.Trigger;
    this.client = node.createClient(Trigger, 'identify_robot');
    node.spin();
    this.logger.log('ROS2 client prêt !');
  }

  async identifyRobot(): Promise<{ success: boolean; message: string }> {
    if (!this.client) {
      this.logger.error('Client ROS2 non initialisé');
      return { success: false, message: 'ROS2 non initialisé' };
    }

    return new Promise((resolve) => {
      const request = new ((rclnodejs.require('std_srvs') as any).srv.Trigger.Request)();
      this.client.sendRequest(request, (response) => {
        if (response) resolve({ success: response.success, message: response.message });
        else resolve({ success: false, message: 'Échec de l’appel ROS2' });
      });
    });
  }
}
