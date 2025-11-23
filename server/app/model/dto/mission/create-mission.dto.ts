import { MissionLogEntry } from '@common/interfaces/mission-log-entry';
import { ApiProperty } from '@nestjs/swagger';
import { ArrayMaxSize, ArrayMinSize, IsArray, IsEnum, IsNumber, IsOptional, IsString, Min } from 'class-validator';
import { OccupancyGrid } from '@common/interfaces/occupancy-grid';


type RobotId = 'limo1' | 'limo2';

export class CreateMissionDto {
    @ApiProperty()
    @IsNumber()
    @Min(0)
    durationSec: number;

    @ApiProperty({ type: [String], description: 'Liste des deux robots impliqués' })
    @IsArray()
    @ArrayMinSize(2)
    @ArrayMaxSize(2)
    @IsString({ each: true })
    robots: string[];

    @ApiProperty({ enum: ['SIMULATION', 'RlogsEAL'] })
    @IsEnum(['SIMULATION', 'REAL'])
    mode: string;

    @ApiProperty()
    @IsNumber()
    @Min(0)
    distance: number;

    @ApiProperty()
    @IsString()
    missionName: string;

    @ApiProperty({ required: false })
    @IsOptional()
    @IsString()
    status?: string;

    @ApiProperty({ required: false, type: [Object] })
    @IsOptional()
    logs?: MissionLogEntry[];

    @ApiProperty({ required: false})
    maps: Record<RobotId, any>;
}
