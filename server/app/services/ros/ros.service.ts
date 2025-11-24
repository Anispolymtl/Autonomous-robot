process.env.ROS_DOMAIN_ID = '66';
import { Injectable, Logger, OnModuleInit } from '@nestjs/common';
import * as rclnodejs from 'rclnodejs';
import { LimoObject } from '@app/interfaces/LimoObject';
import { SocketService } from '@app/services/socket/socket.service';
import { NavService } from '@app/services/nav/nav.service';
import { MappingSerivce } from '@app/services/mapping/mapping.service';
import { StateService } from '@app/services/state/state.service';
import { CodeEditorService } from '@app/services/code-editor/code-editor.service';

type RobotId = 'limo1' | 'limo2';

@Injectable()
export class RosService implements OnModuleInit {
  private readonly logger = new Logger(RosService.name);
  private limoList: LimoObject[];


  constructor(
    private navService: NavService,
    private stateService: StateService,
    private mappingService: MappingSerivce,
    private codeEditor: CodeEditorService
  ) {}

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
    this.navService.initNavService(nodeLimo1, nodeLimo2);
    this.stateService.initStateService();
    this.mappingService.initialiseMappingService();
    this.codeEditor.initCodeEditorService();
    nodeLimo1.spin();
    nodeLimo2.spin();
    this.logger.log('ROS2 client prêt !');
    this.testFetchMissionLogic()
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
  private async testFetchMissionLogic() {
    this.logger.log("🚀 Test de récupération du code de mission...");

    const result = await this.codeEditor.getCode();

    if (result.success) {
      this.logger.log("📄 Code reçu avec succès !");
      this.logger.debug("Aperçu : " + result.code.substring(0, 200) + "...");
    } else {
      this.logger.error("❌ Échec de récupération du code : " + result.message);
    }
  }
}
