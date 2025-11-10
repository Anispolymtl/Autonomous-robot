import { Injectable, Logger } from '@nestjs/common';
import { InjectModel } from '@nestjs/mongoose';
import { Model } from 'mongoose';
import { Mission, MissionDocument } from '@app/model/database/mission';
import { CreateMissionDto } from '@app/model/dto/mission/create-mission.dto';
import { UpdateMissionDto } from '@app/model/dto/mission/update-mission.dto';

@Injectable()
export class MissionDatabaseService {
    private readonly logger = new Logger(MissionDatabaseService.name);

    constructor(
        @InjectModel(Mission.name, 'robot_ops') public missionModel: Model<MissionDocument>,
    ) {}

    async getAllMissions(): Promise<Mission[]> {
        try {
            this.logger.log('Attempting to fetch missions from database...');
            const count = await this.missionModel.countDocuments({});
            this.logger.log(`Total documents in collection: ${count}`);
            
            const missions = await this.missionModel.find({}).sort({ createdAt: -1 }).exec();
            this.logger.log(`Found ${missions.length} missions`);
            
            if (missions.length > 0) {
                this.logger.log(`Sample mission: ${JSON.stringify(missions[0])}`);
            }
            
            return missions;
        } catch (error) {
            this.logger.error(`Error fetching missions: ${error}`);
            this.logger.error(`Error stack: ${error instanceof Error ? error.stack : 'No stack trace'}`);
            throw error;
        }
    }

    async getMissionById(id: string): Promise<Mission | null> {
        return await this.missionModel.findById(id).exec();
    }

    async getMissionsByRobot(robotName: string): Promise<Mission[]> {
        return await this.missionModel.find({ robots: robotName }).sort({ createdAt: -1 }).exec();
    }

    async getMissionsByMode(mode: string): Promise<Mission[]> {
        return await this.missionModel.find({ mode }).sort({ createdAt: -1 }).exec();
    }

    async createMission(mission: CreateMissionDto): Promise<Mission> {
        try {
            const createdMission = await this.missionModel.create(mission);
            return createdMission;
        } catch (error) {
            this.logger.error(`Failed to create mission: ${error}`);
            throw new Error(`Failed to create mission: ${error}`);
        }
    }

    async updateMission(missionDto: UpdateMissionDto): Promise<Mission | null> {
        try {
            const { _id, ...updateData } = missionDto;
            const updatedMission = await this.missionModel.findByIdAndUpdate(
                _id,
                updateData,
                { new: true }
            ).exec();
            return updatedMission;
        } catch (error) {
            this.logger.error(`Failed to update mission: ${error}`);
            throw new Error(`Failed to update mission: ${error}`);
        }
    }

    async deleteMission(id: string): Promise<boolean> {
        try {
            const result = await this.missionModel.findByIdAndDelete(id).exec();
            return result !== null;
        } catch (error) {
            this.logger.error(`Failed to delete mission: ${error}`);
            throw new Error(`Failed to delete mission: ${error}`);
        }
    }

    async getMissionStats(): Promise<{
        total: number;
        byRobot: Record<string, number>;
        byMode: Record<string, number>;
        totalDistance: number;
        averageDuration: number;
    }> {
        const missions = await this.missionModel.find({}).exec();
        
        const stats = {
            total: missions.length,
            byRobot: {} as Record<string, number>,
            byMode: {} as Record<string, number>,
            totalDistance: 0,
            averageDuration: 0,
        };

        missions.forEach(mission => {
            // Par robot
            if (Array.isArray(mission.robots)) {
                mission.robots.forEach((robot) => {
                    if (!robot) return;
                    stats.byRobot[robot] = (stats.byRobot[robot] || 0) + 1;
                });
            }
            
            // Par mode
            if (mission.mode) {
                stats.byMode[mission.mode] = (stats.byMode[mission.mode] || 0) + 1;
            }
            
            // Distance totale
            if (mission.distance) {
                stats.totalDistance += mission.distance;
            }
            
            // Durée moyenne
            if (mission.durationSec) {
                stats.averageDuration += mission.durationSec;
            }
        });

        if (missions.length > 0) {
            stats.averageDuration = Math.round(stats.averageDuration / missions.length);
        }

        return stats;
    }

    async populateDatabase(force: boolean = false): Promise<{ created: number; message: string }> {
        try {
            const count = await this.missionModel.countDocuments({});
            
            if (count > 0 && !force) {
                this.logger.log(`Database already contains ${count} missions. Skipping population. Use force=true to populate anyway.`);
                return { created: 0, message: `Database already contains ${count} missions. Use force=true to populate anyway.` };
            }

            if (force && count > 0) {
                this.logger.log(`Force population: clearing existing ${count} missions...`);
                await this.missionModel.deleteMany({});
            }

            const sampleMissions: CreateMissionDto[] = [
                {
                    missionName: 'Inspection hangar #12',
                    robots: ['Atlas-R2', 'Atlas-R5'],
                    mode: 'SIMULATION',
                    distance: 1500,
                    durationSec: 735,
                },
                {
                    missionName: 'Patrouille secteur A',
                    robots: ['Atlas-R2', 'Atlas-R3'],
                    mode: 'SIMULATION',
                    distance: 2300,
                    durationSec: 1200,
                },
                {
                    missionName: 'Maintenance zone 5',
                    robots: ['Atlas-R1', 'Atlas-R4'],
                    mode: 'REAL',
                    distance: 800,
                    durationSec: 450,
                },
                {
                    missionName: 'Reconnaissance périphérie',
                    robots: ['Atlas-R2', 'Atlas-R4'],
                    mode: 'REAL',
                    distance: 3200,
                    durationSec: 1800,
                },
                {
                    missionName: 'Vérification système',
                    robots: ['Atlas-R1', 'Atlas-R3'],
                    mode: 'REAL',
                    distance: 500,
                    durationSec: 300,
                },
                {
                    missionName: 'Inspection infrastructure',
                    robots: ['Atlas-R2', 'Atlas-R5'],
                    mode: 'REAL',
                    distance: 4500,
                    durationSec: 2400,
                },
            ];

            const created = await this.missionModel.insertMany(sampleMissions);
            this.logger.log(`Successfully populated database with ${created.length} missions`);
            
            return { created: created.length, message: `Successfully created ${created.length} sample missions.` };
        } catch (error) {
            this.logger.error(`Failed to populate database: ${error}`);
            throw new Error(`Failed to populate database: ${error}`);
        }
    }
}
