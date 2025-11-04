import { Logger, Module } from '@nestjs/common';
import { MongooseModule } from '@nestjs/mongoose';
import { ConfigModule, ConfigService } from '@nestjs/config';
import { Mission, missionSchema } from '@app/model/database/mission';
import { MissionDatabaseController } from '@app/controllers/mission-database/mission-database.controller';
import { MissionDatabaseService } from '@app/services/mission-database/mission-database.service';
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
        // Connexion MongoDB pour la base de données robot_ops (Mission)
        MongooseModule.forRootAsync({
            connectionName: 'robot_ops',
            imports: [ConfigModule],
            inject: [ConfigService],
            useFactory: (configService: ConfigService) => {
                const connectionString = configService.get<string>('DATABASE_CONNECTION_STRING') || 'mongodb://localhost:27017/robot_ops';
                // Remplacer le nom de la base de données par robot_ops
                const uri = connectionString.replace(/\/[^/?]+(\?|$)/, '/robot_ops$1');
                return {
                    uri,
                };
            },
        }),
        MongooseModule.forFeature([{ name: Mission.name, schema: missionSchema }], 'robot_ops'),
    ],
    controllers: [DateController, RosController, MissionController, MissionDatabaseController],
    providers: [DateService, ExampleService, Logger, RosService, MissionService, MissionDatabaseService],
})
export class AppModule {}
