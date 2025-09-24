import { Logger, Module } from '@nestjs/common';
import { ConfigModule, ConfigService } from '@nestjs/config';
import { DateService } from '@app/services/date/date.service';
import { RosService } from '@app/services/ros.service';
import { RosController } from '@app/controllers/ros.controller';

@Module({
    imports: [
        ConfigModule.forRoot({ isGlobal: true })
    ],
    controllers: [RosController],
    providers: [DateService, Logger, RosService],
})
export class AppModule {}
