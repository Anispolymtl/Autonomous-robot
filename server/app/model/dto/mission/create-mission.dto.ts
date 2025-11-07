import { ApiProperty } from '@nestjs/swagger';
import { IsEnum, IsNumber, IsString, Min } from 'class-validator';

export class CreateMissionDto {
    @ApiProperty()
    @IsNumber()
    @Min(0)
    durationSec: number;

    @ApiProperty()
    @IsString()
    robotName: string;

    @ApiProperty({ enum: ['SIMULATION', 'REAL'] })
    @IsEnum(['SIMULATION', 'REAL'])
    mode: string;

    @ApiProperty()
    @IsNumber()
    @Min(0)
    distance: number;

    @ApiProperty()
    @IsString()
    missionName: string;
}

