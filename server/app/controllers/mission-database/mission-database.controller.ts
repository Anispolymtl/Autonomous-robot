import { Controller, Get, Post, Patch, Delete, Body, Param, HttpStatus, Res, Query } from '@nestjs/common';
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
    private parseNumberParam(value?: string): number | undefined {
        if (value === undefined) return undefined;
        const parsed = parseInt(value, 10);
        if (Number.isNaN(parsed) || parsed < 0) return undefined;
        return parsed;
    }

    @ApiOkResponse({
        description: 'Returns all missions',
        type: Mission,
        isArray: true,
    })
    @Get('/')
    async getAllMissions(
        @Res() response: Response,
        @Query('limit') limit?: string,
        @Query('skip') skip?: string
    ) {
        try {
            console.log('GET /api/missions - Fetching all missions');
            const parsedLimit = this.parseNumberParam(limit);
            const parsedSkip = this.parseNumberParam(skip);
            const missions = await this.missionDatabaseService.getAllMissions(parsedLimit, parsedSkip);
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
    async getMissionsByRobot(
        @Param('robotName') robotName: string,
        @Res() response: Response,
        @Query('limit') limit?: string,
        @Query('skip') skip?: string
    ) {
        try {
            const parsedLimit = this.parseNumberParam(limit);
            const parsedSkip = this.parseNumberParam(skip);
            const missions = await this.missionDatabaseService.getMissionsByRobot(robotName, parsedLimit, parsedSkip);
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
    async getMissionsByMode(
        @Param('mode') mode: string,
        @Res() response: Response,
        @Query('limit') limit?: string,
        @Query('skip') skip?: string
    ) {
        try {
            const parsedLimit = this.parseNumberParam(limit);
            const parsedSkip = this.parseNumberParam(skip);
            const missions = await this.missionDatabaseService.getMissionsByMode(mode, parsedLimit, parsedSkip);
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

}
