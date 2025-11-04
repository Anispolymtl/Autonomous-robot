import { Controller, Get, Post, Patch, Delete, Body, Param, HttpStatus, Res } from '@nestjs/common';
import { ApiTags, ApiOkResponse, ApiCreatedResponse, ApiNotFoundResponse } from '@nestjs/swagger';
import { Response } from 'express';
import { MissionDatabaseService } from '@app/services/mission-database/mission-database.service';
import { Mission } from '@app/model/database/mission';
import { CreateMissionDto } from '@app/model/dto/mission/create-mission.dto';
import { UpdateMissionDto } from '@app/model/dto/mission/update-mission.dto';

@ApiTags('Missions Database')
@Controller('missions')
export class MissionDatabaseController {
    constructor(private readonly missionDatabaseService: MissionDatabaseService) {}

    @ApiOkResponse({
        description: 'Returns all missions',
        type: Mission,
        isArray: true,
    })
    @Get('/')
    async getAllMissions(@Res() response: Response) {
        try {
            console.log('GET /api/missions - Fetching all missions');
            const missions = await this.missionDatabaseService.getAllMissions();
            console.log(`Returning ${missions.length} missions`);
            response.status(HttpStatus.OK).json(missions);
        } catch (error) {
            console.error('Error in getAllMissions:', error);
            response.status(HttpStatus.INTERNAL_SERVER_ERROR).send(error.message);
        }
    }

    @ApiOkResponse({
        description: 'Get mission by ID',
        type: Mission,
    })
    @ApiNotFoundResponse({
        description: 'Return NOT_FOUND http status when mission does not exist',
    })
    @Get('/:id')
    async getMissionById(@Param('id') id: string, @Res() response: Response) {
        try {
            const mission = await this.missionDatabaseService.getMissionById(id);
            if (!mission) {
                response.status(HttpStatus.NOT_FOUND).send('Mission not found');
                return;
            }
            response.status(HttpStatus.OK).json(mission);
        } catch (error) {
            response.status(HttpStatus.INTERNAL_SERVER_ERROR).send(error.message);
        }
    }

    @ApiOkResponse({
        description: 'Get missions by robot name',
        type: Mission,
        isArray: true,
    })
    @Get('/robot/:robotName')
    async getMissionsByRobot(@Param('robotName') robotName: string, @Res() response: Response) {
        try {
            const missions = await this.missionDatabaseService.getMissionsByRobot(robotName);
            response.status(HttpStatus.OK).json(missions);
        } catch (error) {
            response.status(HttpStatus.INTERNAL_SERVER_ERROR).send(error.message);
        }
    }

    @ApiOkResponse({
        description: 'Get missions by mode',
        type: Mission,
        isArray: true,
    })
    @Get('/mode/:mode')
    async getMissionsByMode(@Param('mode') mode: string, @Res() response: Response) {
        try {
            const missions = await this.missionDatabaseService.getMissionsByMode(mode);
            response.status(HttpStatus.OK).json(missions);
        } catch (error) {
            response.status(HttpStatus.INTERNAL_SERVER_ERROR).send(error.message);
        }
    }

    @ApiOkResponse({
        description: 'Get mission statistics',
    })
    @Get('/stats/overview')
    async getMissionStats(@Res() response: Response) {
        try {
            const stats = await this.missionDatabaseService.getMissionStats();
            response.status(HttpStatus.OK).json(stats);
        } catch (error) {
            response.status(HttpStatus.INTERNAL_SERVER_ERROR).send(error.message);
        }
    }

    @ApiCreatedResponse({
        description: 'Create a new mission',
        type: Mission,
    })
    @Post('/')
    async createMission(@Body() createMissionDto: CreateMissionDto, @Res() response: Response) {
        try {
            const mission = await this.missionDatabaseService.createMission(createMissionDto);
            response.status(HttpStatus.CREATED).json(mission);
        } catch (error) {
            response.status(HttpStatus.BAD_REQUEST).send(error.message);
        }
    }

    @ApiOkResponse({
        description: 'Update a mission',
        type: Mission,
    })
    @ApiNotFoundResponse({
        description: 'Return NOT_FOUND http status when mission does not exist',
    })
    @Patch('/')
    async updateMission(@Body() updateMissionDto: UpdateMissionDto, @Res() response: Response) {
        try {
            const mission = await this.missionDatabaseService.updateMission(updateMissionDto);
            if (!mission) {
                response.status(HttpStatus.NOT_FOUND).send('Mission not found');
                return;
            }
            response.status(HttpStatus.OK).json(mission);
        } catch (error) {
            response.status(HttpStatus.BAD_REQUEST).send(error.message);
        }
    }

    @ApiOkResponse({
        description: 'Delete a mission',
    })
    @ApiNotFoundResponse({
        description: 'Return NOT_FOUND http status when mission does not exist',
    })
    @Delete('/:id')
    async deleteMission(@Param('id') id: string, @Res() response: Response) {
        try {
            const deleted = await this.missionDatabaseService.deleteMission(id);
            if (!deleted) {
                response.status(HttpStatus.NOT_FOUND).json({ message: 'Mission not found' });
                return;
            }
            response.status(HttpStatus.OK).json({ message: 'Mission deleted successfully' });
        } catch (error) {
            response.status(HttpStatus.INTERNAL_SERVER_ERROR).json({ error: error.message });
        }
    }

    @Post('/populate')
    async populateDatabase(@Body() body: { force?: boolean }, @Res() response: Response) {
        try {
            const force = body?.force === true;
            console.log(`POST /api/missions/populate - Populating database (force: ${force})...`);
            const result = await this.missionDatabaseService.populateDatabase(force);
            response.status(HttpStatus.CREATED).json(result);
        } catch (error) {
            console.error('Error populating database:', error);
            response.status(HttpStatus.INTERNAL_SERVER_ERROR).json({ 
                error: error.message || 'Failed to populate database' 
            });
        }
    }
}

