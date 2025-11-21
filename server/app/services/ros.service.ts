process.env.ROS_DOMAIN_ID = '66';
import { Injectable, Logger, OnModuleInit } from '@nestjs/common';
import * as rclnodejs from 'rclnodejs';
import { LimoObject } from '@app/interfaces/LimoObject';
import { SocketService } from './socket/socket.service';
import { NavService } from './nav/nav.service';
import { MappingSerivce } from './mapping/mapping.service';
import { StateService } from '@app/services/state/state.service';

type RobotId = 'limo1' | 'limo2';

@Injectable()
export class RosService implements OnModuleInit {
  private readonly logger = new Logger(RosService.name);
  private limoList: LimoObject[];


  constructor(
    private navService: NavService,
    private stateService: StateService,
    private mappingService: MappingSerivce
  ) {}

  async onModuleInit() {
    await rclnodejs.init();
    const nodeLimo1 = new rclnodejs.Node('identify_client_backend', 'limo1');
    const nodeLimo2 = new rclnodejs.Node('identify_client_backend', 'limo2');
    const Trigger = (rclnodejs.require('std_srvs') as any).srv.Trigger;
    const clientIdLimo1 = nodeLimo1.createClient(Trigger, 'identify_robot');
    const clientIdLimo2 = nodeLimo2.createClient(Trigger, 'identify_robot');
    const returnClient1 = nodeLimo1.createClient(Trigger, 'return_to_base');
    const returnClient2 = nodeLimo2.createClient(Trigger, 'return_to_base');

    this.limoList = [
      { node: nodeLimo1, identifyClient: clientIdLimo1, returnClient: returnClient1 },
      { node: nodeLimo2, identifyClient: clientIdLimo2, returnClient: returnClient2}
    ];
    this.navService.initNavService(nodeLimo1, nodeLimo2);
    this.stateService.initStateService();
    this.mappingService.initialiseMappingService();
    nodeLimo1.spin();
    nodeLimo2.spin();
    this.logger.log('ROS2 client prêt !');
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

  async returnToBase() {
    const Trigger = (rclnodejs.require('std_srvs') as any).srv.Trigger;
    this.limoList.forEach((limo, index) => {
      if (!limo.returnClient) {
        this.logger.error(`Client retour robot ${index + 1} non initialisé`);
        return;
      }

      const request = new Trigger.Request();
      limo.returnClient.sendRequest(request, (response) => {
        if (response?.success) {
          this.logger.log(`Robot ${index + 1} retourne à la base : ${response.message}`);
        } else {
          this.logger.error(
            `Échec du retour pour le robot ${index + 1} : ${response?.message ?? 'réponse vide'}`
          );
        }
      });
    });
  }
}
