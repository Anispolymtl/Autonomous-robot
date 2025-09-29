import { Logger, Module } from '@nestjs/common';
import { MongooseModule } from '@nestjs/mongoose';
import { ConfigModule, ConfigService } from '@nestjs/config';
import { Course, courseSchema } from '@app/model/database/course';
import { CourseController } from '@app/controllers/course/course.controller';
import { CourseService } from '@app/services/course/course.service';
import { DateController } from '@app/controllers/date/date.controller';
import { DateService } from '@app/services/date/date.service';
import { ExampleService } from '@app/services/example/example.service';
import { RosService } from '@app/services/ros.service';
import { RosController } from '@app/controllers/ros.controller';
import { MissionController } from './controllers/mission/mission.controller';
import { MissionService } from './services/misson/mission.service';

@Module({
    imports: [
        ConfigModule.forRoot({ isGlobal: true })
        // MongooseModule.forRootAsync({
        //     imports: [ConfigModule],
        //     inject: [ConfigService],
        //     useFactory: async (config: ConfigService) => ({
        //         uri: config.get<string>('DATABASE_CONNECTION_STRING'), // Loaded from .env
        //     }),
        // }),
        // MongooseModule.forFeature([{ name: Course.name, schema: courseSchema }]),
    ],
    controllers: [DateController, RosController, MissionController], // CourseController, 
    providers: [DateService, ExampleService, Logger, RosService, MissionService], // CourseService,
})
export class AppModule {}
