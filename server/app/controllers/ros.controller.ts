import { Controller, Get } from '@nestjs/common';
import { RosService } from '@app/services/ros.service';

@Controller('identify')
export class RosController {
  constructor(private readonly rosService: RosService) {}

  @Get()
  async identify() {
    console.log('Identification robot demandée');
    return await this.rosService.identifyRobot(0);
  }
}
