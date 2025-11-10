import { Controller, Get } from '@nestjs/common';
import { MissionService } from '@app/services/misson/mission.service';
import { MissionRuntimeService } from '@app/services/mission-runtime/mission-runtime.service';

@Controller('mission')
export class MissionController {
  constructor(
    private readonly missionService: MissionService,
    private readonly missionRuntimeService: MissionRuntimeService
  ) { }

  @Get('/start')
  async start() {
    console.log('Démarrage de la mission demandée');
    return await this.missionService.startMission();
  }

  @Get('/stop')
  async stop() {
    console.log('Arrêt de la mission demandée');
    return await this.missionService.stopMission();
  }

  @Get('/current-mode')
  getCurrentMode() {
    return { mode: this.missionRuntimeService.getCurrentMode() };
  }

  @Get('/active')
  getActiveMission() {
    return this.missionRuntimeService.getActiveMission();
  }
}
