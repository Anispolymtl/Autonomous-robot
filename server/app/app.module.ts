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
// import { MapController } from '@app/controllers/map/map.controller';
// import { MapService } from '@app/services/map/map.service';

@Module({
    imports: [
        ConfigModule.forRoot({ isGlobal: true }),
        MongooseModule.forRootAsync({
            imports: [ConfigModule],
            inject: [ConfigService],
            useFactory: (configService: ConfigService) => {
                const connectionString = configService.get<string>('DATABASE_CONNECTION_STRING') || 'mongodb://localhost:27017/inf3995-106';
                return {
                    uri: connectionString,
                    // Options pour MongoDB Atlas (mongodb+srv://)
                    // Le driver MongoDB natif est utilisé automatiquement avec mongoose 8
                };
            },
        }),
    ],
    controllers: [DateController, RosController, MissionController],
    providers: [DateService, ExampleService, Logger, RosService, MissionService,],
})
export class AppModule {}
