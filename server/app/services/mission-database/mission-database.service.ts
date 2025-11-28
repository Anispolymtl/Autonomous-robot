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

    async getAllMissions(limit?: number, skip?: number): Promise<Mission[]> {
        try {
            this.logger.log('Attempting to fetch missions from database...');
            const count = await this.missionModel.countDocuments({});
            this.logger.log(`Total documents in collection: ${count}`);

            const query = this.missionModel.find({}, { maps: 0 }).sort({ createdAt: -1 });
            if (typeof skip === 'number') query.skip(skip);
            if (typeof limit === 'number') query.limit(limit);

            const missions = await query.exec();
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

    async getMissionsByRobot(robotName: string, limit?: number, skip?: number): Promise<Mission[]> {
        const query = this.missionModel.find({ robots: robotName }, { maps: 0 }).sort({ createdAt: -1 });
        if (typeof skip === 'number') query.skip(skip);
        if (typeof limit === 'number') query.limit(limit);
        return await query.exec();
    }

    async getMissionsByMode(mode: string, limit?: number, skip?: number): Promise<Mission[]> {
        const query = this.missionModel.find({ mode }, { maps: 0 }).sort({ createdAt: -1 });
        if (typeof skip === 'number') query.skip(skip);
        if (typeof limit === 'number') query.limit(limit);
        return await query.exec();
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
        const missions = await this.missionModel.find({}, { maps: 0 }).exec();
        
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

}
