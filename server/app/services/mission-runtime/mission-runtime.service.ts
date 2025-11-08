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
    private missions = new Map<string, MissionRuntimeSnapshot>();
    private socketMissions = new Map<string, Set<string>>();

    createMission(socketId: string, payload: MissionCreatePayload): MissionRuntimeSnapshot {
        if (!payload?.missionName || !payload?.robots?.length) {
            throw new Error('Mission payload incomplet');
        }

        const missionId = randomUUID();
        const snapshot: MissionRuntimeSnapshot = {
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

        this.missions.set(missionId, snapshot);
        this.trackMission(socketId, missionId);
        return snapshot;
    }

    getMission(missionId: string): MissionRuntimeSnapshot | undefined {
        return this.missions.get(missionId);
    }

    updateMission(socketId: string, missionId: string, updates: Partial<Mission>): MissionRuntimeSnapshot {
        const mission = this.getOwnedMission(socketId, missionId);

        if (updates.logs && Array.isArray(updates.logs)) {
            mission.logs = mission.logs ?? [];
            mission.logs.push(...updates.logs as MissionLogEntry[]);
        }

        Object.assign(mission, updates, { updatedAt: new Date() });
        this.missions.set(missionId, mission);
        return mission;
    }

    appendLog(socketId: string, missionId: string, log: MissionLogEntry): MissionRuntimeSnapshot {
        const mission = this.getOwnedMission(socketId, missionId);
        mission.logs = mission.logs ?? [];
        mission.logs.push({
            ...log,
            timestamp: log.timestamp ?? new Date().toISOString(),
        });
        mission.updatedAt = new Date();
        this.missions.set(missionId, mission);
        return mission;
    }

    completeMission(socketId: string, missionId: string): MissionRuntimeSnapshot {
        const mission = this.getOwnedMission(socketId, missionId);
        this.missions.delete(missionId);
        this.untrackMission(socketId, missionId);
        mission.status = 'COMPLETED';
        mission.updatedAt = new Date();
        return mission;
    }

    clearMissionsForSocket(socketId: string): void {
        const ids = this.socketMissions.get(socketId);
        if (!ids) return;
        ids.forEach((missionId) => this.missions.delete(missionId));
        this.socketMissions.delete(socketId);
    }

    private trackMission(socketId: string, missionId: string): void {
        if (!this.socketMissions.has(socketId)) {
            this.socketMissions.set(socketId, new Set());
        }
        this.socketMissions.get(socketId)?.add(missionId);
    }

    private untrackMission(socketId: string, missionId: string): void {
        const missions = this.socketMissions.get(socketId);
        missions?.delete(missionId);
        if (missions && missions.size === 0) {
            this.socketMissions.delete(socketId);
        }
    }

    private getOwnedMission(socketId: string, missionId: string): MissionRuntimeSnapshot {
        const mission = this.missions.get(missionId);
        if (!mission) {
            throw new Error('Mission introuvable');
        }
        if (mission.socketId !== socketId) {
            throw new Error('Accès refusé à cette mission');
        }
        return mission;
    }
}

