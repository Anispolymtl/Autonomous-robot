import { MissionLogEntry as SharedMissionLogEntry } from '@common/interfaces/mission-log-entry';

export type MissionLogEntry = SharedMissionLogEntry;

export interface Mission {
    _id?: string;
    createdAt?: Date | string;
    durationSec: number;
    robots: string[];
    mode: 'SIMULATION' | 'REAL';
    distance: number;
    missionName: string;
    logs?: MissionLogEntry[];
    status?: string;
}

export interface CreateMissionDto {
    durationSec: number;
    robots: string[];
    mode: 'SIMULATION' | 'REAL';
    distance: number;
    missionName: string;
    logs?: MissionLogEntry[];
    status?: string;
}

export interface UpdateMissionDto {
    _id: string;
    durationSec?: number;
    robots?: string[];
    mode?: 'SIMULATION' | 'REAL';
    distance?: number;
    missionName?: string;
    logs?: MissionLogEntry[];
    status?: string;
}

export interface MissionStats {
    total: number;
    byRobot: Record<string, number>;
    byMode: Record<string, number>;
    totalDistance: number;
    averageDuration: number;
}
