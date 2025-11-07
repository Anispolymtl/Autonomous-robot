import { Logger, Module } from '@nestjs/common';
import { MongooseModule } from '@nestjs/mongoose';
import { ConfigModule, ConfigService } from '@nestjs/config';
import { Course, courseSchema } from '@app/model/database/course';
import { RosService } from '@app/services/ros.service';
import { RosController } from '@app/controllers/ros.controller';
import { MissionController } from './controllers/mission/mission.controller';
import { MissionService } from './services/misson/mission.service';
import { SocketService } from './services/socket/socket.service';

@Module({
    imports: [
        ConfigModule.forRoot({ isGlobal: true }),
    ],
    controllers: [RosController, MissionController],
    providers: [Logger, RosService, MissionService, SocketService],
})
export class AppModule {}
