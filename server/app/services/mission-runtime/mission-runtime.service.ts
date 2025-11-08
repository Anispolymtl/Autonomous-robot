import { Injectable } from '@nestjs/common';
import { Mission } from '@app/model/database/mission';
import { randomUUID } from 'crypto';

export interface MissionLogEntry {
    timestamp?: string | number;
    level?: string;
    message?: string;
    phase?: string;
    details?: string;
    [key: string]: unknown;
}

export interface MissionRuntimeSnapshot extends Mission {
    missionId: string;
    socketId: string;
    createdAt?: Date;
    updatedAt?: Date;
    logs?: MissionLogEntry[];
}

export interface MissionCreatePayload {
    missionName: string;
    robots: string[];
    mode: 'SIMULATION' | 'REAL';
    distance?: number;
    durationSec?: number;
    status?: string;
    logs?: MissionLogEntry[];
}

@Injectable()
export class MissionRuntimeService {
    private activeMission: MissionRuntimeSnapshot | null = null;
    private currentMode: 'SIMULATION' | 'REAL' | null = null;

    createMission(socketId: string, payload: MissionCreatePayload): MissionRuntimeSnapshot {
        if (this.activeMission) {
            throw new Error('Une mission est déjà en cours');
        }

        if (!payload?.missionName || !payload?.robots?.length) {
            throw new Error('Mission payload incomplet');
        }

        const missionId = randomUUID();
        this.activeMission = {
            missionId,
            socketId,
            missionName: payload.missionName,
            robots: payload.robots,
            mode: payload.mode ?? 'SIMULATION',
            distance: payload.distance ?? 0,
            durationSec: payload.durationSec ?? 0,
            status: payload.status ?? 'PENDING',
            logs: payload.logs ?? [],
            createdAt: new Date(),
            updatedAt: new Date(),
        };

        this.currentMode = (this.activeMission.mode as 'SIMULATION' | 'REAL' | undefined) ?? null;
        return this.activeMission;
    }

    getActiveMission(): MissionRuntimeSnapshot | null {
        return this.activeMission;
    }

    updateMission(missionId: string, updates: Partial<Mission>): MissionRuntimeSnapshot {
        const mission = this.ensureMission(missionId);

        if (updates.logs && Array.isArray(updates.logs)) {
            mission.logs = mission.logs ?? [];
            mission.logs.push(...(updates.logs as MissionLogEntry[]));
            delete updates.logs;
        }

        Object.assign(mission, updates, { updatedAt: new Date() });
        this.activeMission = mission;
        return mission;
    }

    appendLog(missionId: string, log: MissionLogEntry): MissionRuntimeSnapshot {
        const mission = this.ensureMission(missionId);
        mission.logs = mission.logs ?? [];
        mission.logs.push({
            ...log,
            timestamp: log.timestamp ?? new Date().toISOString(),
        });
        mission.updatedAt = new Date();
        this.activeMission = mission;
        return mission;
    }

    completeMission(missionId: string): MissionRuntimeSnapshot {
        const mission = this.ensureMission(missionId);
        mission.status = 'COMPLETED';
        mission.updatedAt = new Date();
        this.activeMission = null;
        this.currentMode = null;
        return mission;
    }

    clearMissionForSocket(socketId: string): void {
        if (this.activeMission?.socketId === socketId) {
            this.activeMission = null;
            this.currentMode = null;
        }
    }

    getCurrentMode(): 'SIMULATION' | 'REAL' | null {
        return this.currentMode;
    }

    private ensureMission(missionId: string): MissionRuntimeSnapshot {
        if (!this.activeMission || this.activeMission.missionId !== missionId) {
            throw new Error('Mission introuvable');
        }
        return this.activeMission;
    }
}
