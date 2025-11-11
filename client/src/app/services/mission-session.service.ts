import { Injectable } from '@angular/core';
import { firstValueFrom } from 'rxjs';
import { Mission, MissionLogEntry } from '@app/interfaces/mission';
import { SocketService } from '@app/services/socket.service';
import { MissionMode, MissionModeService, ActiveMissionResponse } from '@app/services/mission-mode.service';

interface MissionCreatedPayload {
    missionId: string;
    mission: Mission;
}

interface MissionEventPayload {
    missionId: string;
    mission: Mission;
}

interface MissionErrorPayload {
    missionId?: string;
    message: string;
}

@Injectable({
    providedIn: 'root',
})
export class MissionSessionService {
    private missionId: string | null = null;
    private missionSnapshot: Mission | null = null;
    private missionStartTimestamp: number | null = null;
    private readonly defaultRobots: string[] = ['limo1', 'limo2'];
    private listenersRegistered = false;
    private missionUpdateHandler = (...args: unknown[]) => {
        const payload = args[0] as MissionEventPayload;
        if (this.missionId && payload?.missionId === this.missionId) {
            this.missionSnapshot = payload.mission;
        }
    };

    constructor(
        private readonly socketService: SocketService,
        private readonly missionModeService: MissionModeService
    ) {}

    get currentMission(): Mission | null {
        return this.missionSnapshot;
    }

    get hasActiveMission(): boolean {
        return Boolean(this.missionId);
    }

    async initializeMission(missionName: string, mode: MissionMode): Promise<MissionCreatedPayload> {
        await this.ensureSocketConnected();
        this.registerMissionListeners();

        return new Promise<MissionCreatedPayload>((resolve, reject) => {
            const handleCreated = (...args: unknown[]) => {
                const payload = args[0] as MissionCreatedPayload;
                cleanup();
                this.missionId = payload.missionId;
                this.missionSnapshot = payload.mission;
                this.missionModeService.setMode(payload.mission?.mode ?? null);
                resolve(payload);
            };

            const handleError = (...args: unknown[]) => {
                const payload = args[0] as MissionErrorPayload;
                cleanup();
                reject(new Error(payload.message));
            };

            const cleanup = () => {
                this.socketService.off('mission:created', handleCreated);
                this.socketService.off('mission:error', handleError);
            };

            this.socketService.on('mission:created', handleCreated);
            this.socketService.on('mission:error', handleError);

            const payload: Mission = {
                missionName,
                robots: this.defaultRobots,
                mode: mode ?? 'SIMULATION',
                distance: 0,
                durationSec: 0,
                logs: [],
                status: 'PENDING',
            };

            this.socketService.send('mission:create', payload);
        });
    }

    markMissionStarted(): void {
        if (!this.missionId) return;
        this.missionStartTimestamp = Date.now();
        this.sendMissionUpdate({ status: 'RUNNING' });
    }

    sendMissionUpdate(data: Partial<Mission>): void {
        if (!this.missionId) return;
        if (data.durationSec === undefined && this.missionStartTimestamp) {
            const durationSec = Math.max(0, Math.round((Date.now() - this.missionStartTimestamp) / 1000));
            data = { ...data, durationSec };
        }

        this.socketService.send('mission:update', {
            missionId: this.missionId,
            data,
        });
    }

    async rehydrateActiveMission(): Promise<void> {
        if (this.missionId) return;
        const mission: ActiveMissionResponse | null = await firstValueFrom(this.missionModeService.fetchActiveMission());
        if (!mission) return;
        this.missionId = mission.missionId;
        this.missionSnapshot = mission;
        if (mission.durationSec) {
            this.missionStartTimestamp = Date.now() - mission.durationSec * 1000;
        }
        this.missionModeService.setMode(mission.mode ?? null);
    }

    appendLog(entry: Partial<MissionLogEntry>): void {
        if (!this.missionId) return;
        const normalized = this.normalizeLogEntry(entry);
        this.socketService.send('mission:add-log', {
            missionId: this.missionId,
            log: normalized,
        });
    }

    async completeMission(): Promise<Mission | null> {
        if (!this.missionId) return null;

        const finalDuration = this.missionStartTimestamp
            ? Math.max(0, Math.round((Date.now() - this.missionStartTimestamp) / 1000))
            : undefined;
        this.sendMissionUpdate({
            status: 'COMPLETED',
            durationSec: finalDuration,
        });

        return new Promise<Mission | null>((resolve, reject) => {
            const handleFinalized = (...args: unknown[]) => {
                const payload = args[0] as MissionEventPayload;
                cleanup();
                this.missionSnapshot = payload.mission;
                const mission = { ...payload.mission };
                this.resetSession();
                resolve(mission);
            };

            const handleError = (...args: unknown[]) => {
                const payload = args[0] as MissionErrorPayload;
                cleanup();
                reject(new Error(payload.message));
            };

            const cleanup = () => {
                this.socketService.off('mission:finalized', handleFinalized);
                this.socketService.off('mission:error', handleError);
            };

            this.socketService.on('mission:finalized', handleFinalized);
            this.socketService.on('mission:error', handleError);
            this.socketService.send('mission:complete', { missionId: this.missionId });
        });
    }

    disconnectSocket(): void {
        if (this.socketService.isSocketAlive()) {
            this.socketService.disconnect();
        }
    }

    private async ensureSocketConnected(): Promise<void> {
        if (this.socketService.isSocketAlive()) return;

        this.socketService.connect('client');
        await new Promise<void>((resolve, reject) => {
            const timeout = setTimeout(() => {
                cleanup();
                reject(new Error('Connexion socket indisponible'));
            }, 5000);

            const handleConnect = () => {
                cleanup();
                resolve();
            };

            const handleError = (...args: unknown[]) => {
                const err = (args[0] as Error) ?? new Error('Unknown error');
                cleanup();
                reject(err);
            };

            const cleanup = () => {
                clearTimeout(timeout);
                this.socketService.off('connect', handleConnect);
                this.socketService.off('connect_error', handleError);
            };

            this.socketService.on('connect', handleConnect);
            this.socketService.on('connect_error', handleError);
        });
    }

    private registerMissionListeners(): void {
        if (this.listenersRegistered) return;

        this.socketService.on('mission:updated', this.missionUpdateHandler);

        this.listenersRegistered = true;
    }

    private resetSession(): void {
        this.missionId = null;
        this.missionSnapshot = null;
        this.missionStartTimestamp = null;
        if (this.listenersRegistered) {
            this.socketService.off('mission:updated', this.missionUpdateHandler);
            this.listenersRegistered = false;
        }
        this.disconnectSocket();
        this.missionModeService.setMode(null);
    }

    private normalizeLogEntry(entry: Partial<MissionLogEntry> | undefined): MissionLogEntry {
        const timestamp = entry?.timestamp ?? new Date().toISOString();
        const robot = entry?.robot ?? this.defaultRobots[0];
        const category = entry?.category ?? 'Command';
        const action = entry?.action ?? 'log';
        const details = { ...(entry?.details ?? {}) };

        return {
            timestamp,
            robot,
            category,
            action,
            details,
        };
    }
}
