import { Prop, Schema, SchemaFactory } from '@nestjs/mongoose';
import { ApiProperty } from '@nestjs/swagger';
import { Document } from 'mongoose';

export type MissionDocument = Mission & Document;

@Schema({ collection: 'missions', timestamps: true, strict: false })
export class Mission {
    @ApiProperty()
    _id?: string;

    @ApiProperty()
    @Prop({ required: false })
    createdAt?: Date;

    @ApiProperty()
    @Prop({ required: false })
    durationSec?: number;

    @ApiProperty({ type: [String], description: 'Liste des robots impliqués (2 entrées)' })
    @Prop({ type: [String], required: true, validate: [(arr: string[]) => Array.isArray(arr) && arr.length === 2, 'Robots array must contain exactly 2 entries'] })
    robots: string[];

    @ApiProperty()
    @Prop({ required: false, enum: ['SIMULATION', 'REAL'] })
    mode?: string;

    @ApiProperty()
    @Prop({ required: false })
    distance?: number;

    @ApiProperty()
    @Prop({ required: false })
    missionName?: string;

    @ApiProperty({ required: false })
    @Prop({ required: false })
    status?: string;

    @ApiProperty({ required: false, type: [Object] })
    @Prop({ type: Array, default: [] })
    logs?: Record<string, unknown>[];
}

export const missionSchema = SchemaFactory.createForClass(Mission);
