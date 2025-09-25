import { Controller, Get, Query } from '@nestjs/common';
import { RosService } from '@app/services/ros.service';

@Controller('identify')
export class RosController {
  constructor(private readonly rosService: RosService) {}

  @Get()
  async identify(@Query('robotId') robotId: number) {
    console.log('Identification robot demandée par robot', robotId);
    return await this.rosService.identifyRobot(robotId);
  }
}
