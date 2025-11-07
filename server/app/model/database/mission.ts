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

    @ApiProperty()
    @Prop({ required: false })
    robotName?: string;

    @ApiProperty()
    @Prop({ required: false, enum: ['SIMULATION', 'REAL'] })
    mode?: string;

    @ApiProperty()
    @Prop({ required: false })
    distance?: number;

    @ApiProperty()
    @Prop({ required: false })
    missionName?: string;
}

export const missionSchema = SchemaFactory.createForClass(Mission);

