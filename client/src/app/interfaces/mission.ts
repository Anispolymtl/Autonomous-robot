export interface Mission {
    _id?: string;
    createdAt?: Date | string;
    durationSec: number;
    robotName: string;
    mode: 'SIMULATION' | 'REAL';
    distance: number;
    missionName: string;
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

