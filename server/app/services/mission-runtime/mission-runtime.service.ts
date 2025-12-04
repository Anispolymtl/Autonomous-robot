import { Injectable } from '@nestjs/common';
import { Mission } from '@app/model/database/mission';
import { randomUUID } from 'crypto';
import { MissionLogEntry } from '@common/interfaces/mission-log-entry';
import { MissionDatabaseService } from '../mission-database/mission-database.service';
import { CreateMissionDto } from '@app/model/dto/mission/create-mission.dto';
import { SocketService } from '../socket/socket.service';

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
    constructor(
        private databaseService: MissionDatabaseService,
        private socketService: SocketService
    ){}
    private activeMission: MissionRuntimeSnapshot | null = null;
    private currentMode: 'SIMULATION' | 'REAL' | null = null;

    createMission(socketId: string, payload: MissionCreatePayload): MissionRuntimeSnapshot {
        if (this.activeMission) {
            throw new Error('Une mission est déjà en cours');
        }

        if (!payload?.missionName || !payload?.robots?.length) {
            throw new Error('Mission payload incomplet');
        }

        const normalizedLogs = payload.logs?.map((log) => this.normalizeLogEntry(log)) ?? [];
        const initialDistance = normalizedLogs.length
            ? this.computeTotalDistance(normalizedLogs)
            : payload.distance ?? 0;
        const missionId = randomUUID();
        this.activeMission = {
            missionId,
            socketId,
            missionName: payload.missionName,
            robots: payload.robots,
            mode: payload.mode ?? 'SIMULATION',
            distance: initialDistance,
            durationSec: payload.durationSec ?? 0,
            status: payload.status ?? 'PENDING',
            logs: normalizedLogs,
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
            mission.logs.push(...(updates.logs as MissionLogEntry[]).map((log) => this.normalizeLogEntry(log)));
            delete updates.logs;
            mission.distance = this.computeTotalDistance(mission.logs);
        }

        Object.assign(mission, updates, { updatedAt: new Date() });
        this.activeMission = mission;
        return mission;
    }

    appendLog(missionId: string, log: MissionLogEntry): MissionRuntimeSnapshot {
        const mission = this.ensureMission(missionId);
        mission.logs = mission.logs ?? [];
        mission.logs.push(this.normalizeLogEntry(log));
        mission.distance = this.computeTotalDistance(mission.logs);
        mission.updatedAt = new Date();
        this.activeMission = mission;
        return mission;
    }

    completeMission(missionId: string): MissionRuntimeSnapshot {
        const mission = this.ensureMission(missionId);
        mission.status = 'COMPLETED';
        mission.distance = this.computeTotalDistance(mission.logs);
        mission.updatedAt = new Date();
        this.activeMission = null;
        this.currentMode = null;
        const missionCreateObj: CreateMissionDto = {
            missionName: mission.missionName,
            robots: mission.robots,
            mode: mission.mode,
            distance: mission.distance ?? 0,
            durationSec: mission.durationSec ?? 0,
            status: mission.status,
            logs: mission.logs ?? [],
            maps: this.socketService.getMaps()
        }
        this.databaseService.createMission(missionCreateObj);
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

    private normalizeLogEntry(entry: Partial<MissionLogEntry>): MissionLogEntry {
        return {
            timestamp: entry.timestamp ?? new Date().toISOString(),
            robot: entry.robot ?? 'unknown',
            category: entry.category ?? 'Command',
            action: entry.action ?? 'log',
            details: { ...(entry.details ?? {}) },
        };
    }

    private computeTotalDistance(logs: MissionLogEntry[] | undefined): number {
        if (!logs?.length) {
            return 0;
        }

        const maxDistanceByRobot = new Map<string, number>();

        for (const log of logs) {
            if (!log) continue;
            const rawDistance = log.details?.['totalDistance'];
            if (rawDistance === undefined || rawDistance === null) continue;

            const distance = typeof rawDistance === 'number' ? rawDistance : Number(rawDistance);
            if (!Number.isFinite(distance) || distance < 0) continue;

            const robotId = log.robot ?? 'unknown';
            const currentMax = maxDistanceByRobot.get(robotId) ?? 0;
            if (distance > currentMax) {
                maxDistanceByRobot.set(robotId, distance);
            }
        }

        const total = Array.from(maxDistanceByRobot.values()).reduce((sum, value) => sum + value, 0);
        return Number(total.toFixed(3));
    }
}
