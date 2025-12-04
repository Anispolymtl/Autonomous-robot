process.env.ROS_DOMAIN_ID = '66';
import { Injectable, Logger, OnModuleInit } from '@nestjs/common';
import * as rclnodejs from 'rclnodejs';
import { LimoObject } from '@app/interfaces/LimoObject';
import { NavService } from '@app/services/nav/nav.service';
import { MappingSerivce } from '@app/services/mapping/mapping.service';
import { StateService } from '@app/services/state/state.service';
import { CodeEditorService } from '@app/services/code-editor/code-editor.service';
import { SocketService } from '../socket/socket.service';

type RobotId = 'limo1' | 'limo2';

@Injectable()
export class RosService implements OnModuleInit {
  private readonly logger = new Logger(RosService.name);
  private limoList: LimoObject[];

  private sleep(ms: number) {
    return new Promise((resolve) => setTimeout(resolve, ms));
  }


  constructor(
    private navService: NavService,
    private stateService: StateService,
    private mappingService: MappingSerivce,
    private codeEditor: CodeEditorService,
    private socketService: SocketService
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
      { node: nodeLimo2, identifyClient: clientIdLimo2, returnClient: returnClient2 }
    ];
    this.navService.initNavService(nodeLimo1, nodeLimo2);
    this.stateService.initStateService();
    this.mappingService.initialiseMappingService();
    this.codeEditor.initCodeEditorService();
    this.startMissionListners('limo1');
    this.startMissionListners('limo2');
    nodeLimo1.spin();
    nodeLimo2.spin();
    this.logger.log('ROS2 client prêt !');
  }

  async identifyRobot(id: number): Promise<{ success: boolean; message: string }> {
    const obj = this.limoList[id - 1];
    if (!obj?.identifyClient) {
      this.logger.error('Client ROS2 non initialisé');
      return { success: false, message: 'ROS2 non initialisé' };
    }

    const Trigger = (rclnodejs.require('std_srvs') as any).srv.Trigger;
    const robotLabel = id === 1 ? 'limo1' : 'limo2';
    const maxAttempts = 3;

    for (let attempt = 1; attempt <= maxAttempts; attempt++) {
      try {
        await obj.identifyClient.waitForService(1000);

        const request = new Trigger.Request();
        const response: any = await new Promise((resolve) =>
          obj.identifyClient.sendRequest(request, (resp) => resolve(resp))
        );

        if (response?.success) {
          this.logger.log(`[${robotLabel}] Identification réussie : ${response.message}`);
          return { success: true, message: response.message };
        }

        this.logger.warn(`[${robotLabel}] Identification tentative ${attempt}/${maxAttempts} échouée: ${response?.message ?? 'réponse vide'}`);
      } catch (err) {
        this.logger.warn(`[${robotLabel}] identify_robot tentative ${attempt}/${maxAttempts} erreur: ${(err as Error).message}`);
      }

      if (attempt < maxAttempts) await this.sleep(300);
    }

    return { success: false, message: `Échec de l’identification pour ${robotLabel}` };
  }

  async returnToBase() {
    const Trigger = (rclnodejs.require('std_srvs') as any).srv.Trigger;
    // Marquer le retour en cours pour ne pas écraser l'état côté UI
    this.navService.setReturnInProgress('limo1', true);
    this.navService.setReturnInProgress('limo2', true);

    // Stoppe les navigations en cours pour éviter les conflits avec le retour
    await Promise.all([
      this.navService.cancelNavigation('limo1'),
      this.navService.cancelNavigation('limo2'),
    ]);

    // Laisser un petit temps pour que Nav2 libère ses actions avant de demander le retour
    await this.sleep(200);

    // Envoie le retour, avec retries si la stack n'est pas encore libérée
    await Promise.all(
      this.limoList.map(async (limo, index) => {
        const robotId = index === 0 ? 'limo1' : 'limo2';
        if (!limo.returnClient) {
          this.logger.error(`Client retour robot ${index + 1} non initialisé`);
          this.navService.setReturnInProgress(robotId, false);
          return;
        }

        const request = new Trigger.Request();
        const maxAttempts = 3;
        for (let attempt = 1; attempt <= maxAttempts; attempt++) {
          try {
            await limo.returnClient.waitForService(1000);
            const response: any = await new Promise((resolve) =>
              limo.returnClient.sendRequest(request, (resp) => resolve(resp))
            );
            if (response?.success) {
              this.logger.log(`Robot ${index + 1} retourne à la base : ${response.message}`);
              break;
            }
            this.logger.warn(`Retour à la base robot ${index + 1} tentative ${attempt}/${maxAttempts} échouée: ${response?.message ?? 'réponse vide'}`);
          } catch (err) {
            this.logger.warn(`Retour à la base robot ${index + 1} tentative ${attempt}/${maxAttempts} erreur: ${(err as Error).message}`);
          }
          if (attempt < maxAttempts) await this.sleep(300);
        }

        // Libère le flag retour après les tentatives
        this.navService.setReturnInProgress(robotId, false);
      })
    );
  }

  startMissionListners(robotId: RobotId) {
    const missionNode = new rclnodejs.Node(`exploration_listener`, robotId);
    missionNode.createSubscription(
      'std_msgs/msg/String',
      `exploration_debug`,
      (msg) => {
        this.socketService.sendExplorationDebugToAllSockets(msg, robotId);
      }
    );
    missionNode.createSubscription(
      'std_msgs/msg/String',
      `exploration_step`,
      (msg) => {
        this.socketService.sendExplorationStepToAllSockets(msg, robotId);
      }
    );
    missionNode.createSubscription(
      'geometry_msgs/msg/Point',
      `candidate_frontier`,
      (msg) => {
        this.socketService.sendExplorationCandidateToAllSockets(msg, robotId);
      }
    );
    missionNode.spin();      
  }
}
