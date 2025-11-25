import { Controller, Get, Query } from '@nestjs/common';
import { RosService } from '@app/services/ros/ros.service';

@Controller('identify')
export class RosController {
  constructor(private readonly rosService: RosService) { }

  @Get()
  async identify(@Query('id') id: number) {
    console.log('Identification robot demandée par robot', id);
    if (id === undefined || id < 1 || id > 2) {
      return { success: false, message: 'ID de robot invalide' };
    }
    return await this.rosService.identifyRobot(id);
  }
}
