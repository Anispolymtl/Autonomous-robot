import { Controller, Get } from '@nestjs/common';
import { MissionService } from '@app/services/misson/mission.service';

@Controller('mission')
export class MissionController {
  constructor(private readonly missionService: MissionService) { }

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
}
