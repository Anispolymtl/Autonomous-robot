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
    controllers: [DateController, RosController], // CourseController, 
    providers: [DateService, ExampleService, Logger, RosService], // CourseService,
})
export class AppModule {}
