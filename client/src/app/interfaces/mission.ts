export type MissionLogEntry = MissionLogObject | MissionLogPrimitive;

export type MissionLogPrimitive = string | number | boolean | null | undefined;

export interface MissionLogObject {
    timestamp?: Date | string | number;
    level?: string;
    phase?: string;
    message?: string;
    details?: string;
    [key: string]: unknown;
}

export interface Mission {
    _id?: string;
    createdAt?: Date | string;
    durationSec: number;
    robotName: string;
    mode: 'SIMULATION' | 'REAL';
    distance: number;
    missionName: string;
    logs?: MissionLogEntry[];
}

export interface CreateMissionDto {
    durationSec: number;
    robotName: string;
    mode: 'SIMULATION' | 'REAL';
    distance: number;
    missionName: string;
}

export interface UpdateMissionDto {
    _id: string;
    durationSec?: number;
    robotName?: string;
    mode?: 'SIMULATION' | 'REAL';
    distance?: number;
    missionName?: string;
}

export interface MissionStats {
    total: number;
    byRobot: Record<string, number>;
    byMode: Record<string, number>;
    totalDistance: number;
    averageDuration: number;
}
