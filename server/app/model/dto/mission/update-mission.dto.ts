import { ApiProperty } from '@nestjs/swagger';
import { IsEnum, IsNumber, IsOptional, IsString, Min } from 'class-validator';

export class UpdateMissionDto {
    @ApiProperty()
    @IsString()
    _id: string;

    @ApiProperty({ required: false })
    @IsOptional()
    @IsNumber()
    @Min(0)
    durationSec?: number;

    @ApiProperty({ required: false })
    @IsOptional()
    @IsString()
    robotName?: string;

    @ApiProperty({ enum: ['SIMULATION', 'REAL'], required: false })
    @IsOptional()
    @IsEnum(['SIMULATION', 'REAL'])
    mode?: string;

    @ApiProperty({ required: false })
    @IsOptional()
    @IsNumber()
    @Min(0)
    distance?: number;

    @ApiProperty({ required: false })
    @IsOptional()
    @IsString()
    missionName?: string;
}

