import { ApiProperty } from '@nestjs/swagger';
import { ArrayMaxSize, ArrayMinSize, IsArray, IsEnum, IsNumber, IsOptional, IsString, Min } from 'class-validator';

export class UpdateMissionDto {
    @ApiProperty()
    @IsString()
    _id: string;

    @ApiProperty({ required: false })
    @IsOptional()
    @IsNumber()
    @Min(0)
    durationSec?: number;

    @ApiProperty({ required: false, type: [String] })
    @IsOptional()
    @IsArray()
    @ArrayMinSize(2)
    @ArrayMaxSize(2)
    @IsString({ each: true })
    robots?: string[];

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

    @ApiProperty({ required: false })
    @IsOptional()
    @IsString()
    status?: string;

    @ApiProperty({ required: false, type: [Object] })
    @IsOptional()
    logs?: Record<string, unknown>[];
}
