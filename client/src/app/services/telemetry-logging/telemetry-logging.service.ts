import { Injectable, OnDestroy } from '@angular/core';
import { MissionSessionService } from '@app/services/mission-session/mission-session.service';
import { SocketService } from '@app/services/socket/socket.service';
import { PoseData } from '@app/interfaces/pose-data';
import { Socket } from 'socket.io-client';

type RobotId = 'limo1' | 'limo2';

interface PoseUpdatePayload {
    robot: string;
    poseData: PoseData;
}

@Injectable({
    providedIn: 'root',
})
export class TelemetryLoggingService implements OnDestroy {
    private readonly robots: RobotId[] = ['limo1', 'limo2'];
    private poseCache: Record<RobotId, PoseData | null> = {
        limo1: null,
        limo2: null,
    };
    private lastLoggedAt: Record<RobotId, number> = {
        limo1: 0,
        limo2: 0,
    };
    private totalDistance: Record<RobotId, number> = {
        limo1: 0,
        limo2: 0,
    };
    private monitorInterval?: ReturnType<typeof setInterval>;
    private attachedSocket?: Socket;
    private readonly poseListener = (payload: PoseUpdatePayload) => this.handlePoseUpdate(payload);
    private readonly disconnectListener = () => {
        this.attachedSocket = undefined;
    };
    private missionActive = false;

    constructor(
        private readonly missionSessionService: MissionSessionService,
        private readonly socketService: SocketService
    ) {}

    start(): void {
        if (this.monitorInterval) return;
        this.monitorInterval = setInterval(() => this.tick(), 1000);
    }

    ngOnDestroy(): void {
        this.teardown();
    }

    private tick(): void {
        const hasMission = this.missionSessionService.hasActiveMission;
        if (hasMission && !this.missionActive) {
            this.resetTelemetry();
        } else if (!hasMission && this.missionActive) {
            this.resetTelemetry();
        }
        this.missionActive = hasMission;

        if (!hasMission) return;

        this.ensureSocketListeners();
        this.flushSensorLogs();
    }

    private ensureSocketListeners(): void {
        if (!this.socketService.isSocketAlive()) {
            this.socketService.connect('client');
        }

        const socket = this.socketService.getSocket;
        if (!socket) return;

        if (this.attachedSocket === socket) return;

        if (this.attachedSocket && this.attachedSocket !== socket) {
            this.attachedSocket.off('poseUpdate', this.poseListener);
            this.attachedSocket.off('disconnect', this.disconnectListener);
        }

        socket.on('poseUpdate', this.poseListener);
        socket.on('disconnect', this.disconnectListener);
        this.attachedSocket = socket;
    }

    private handlePoseUpdate(payload: PoseUpdatePayload): void {
        if (!payload?.poseData || !payload.robot) return;
        if (!this.missionSessionService.hasActiveMission) return;

        const robot = this.normalizeRobotId(payload.robot);
        if (!robot) return;

        const pose = payload.poseData;
        const previousPose = this.poseCache[robot];
        if (previousPose) {
            this.totalDistance[robot] += this.computeDelta(previousPose, pose);
        }
        this.poseCache[robot] = pose;
    }

    private flushSensorLogs(): void {
        const now = Date.now();
        this.robots.forEach((robot) => {
            const pose = this.poseCache[robot];
            if (!pose) return;

            if (now - this.lastLoggedAt[robot] < 1000) return;

            const { x, y, z } = pose.pose.position;
            const headingRad = this.quaternionToYaw(pose.pose.orientation);
            const distanceFromOrigin = Math.sqrt(x * x + y * y + z * z);
            const totalDistance = Number(this.totalDistance[robot].toFixed(3));

            this.missionSessionService.appendLog({
                category: 'Sensor',
                robot,
                action: 'pose_sensor_reading',
                details: {
                    posX: Number(x.toFixed(3)),
                    posY: Number(y.toFixed(3)),
                    posZ: Number(z.toFixed(3)),
                    headingDeg: Number((headingRad * (180 / Math.PI)).toFixed(1)),
                    distanceFromOrigin: Number(distanceFromOrigin.toFixed(3)),
                    totalDistance,
                },
            });

            this.missionSessionService.updateRobotDistance(robot, totalDistance);
            this.lastLoggedAt[robot] = now;
        });
    }

    private resetTelemetry(): void {
        this.poseCache = { limo1: null, limo2: null };
        this.lastLoggedAt = { limo1: 0, limo2: 0 };
        this.totalDistance = { limo1: 0, limo2: 0 };
    }

    private teardown(): void {
        if (this.monitorInterval) {
            clearInterval(this.monitorInterval);
            this.monitorInterval = undefined;
        }
        if (this.attachedSocket) {
            this.attachedSocket.off('poseUpdate', this.poseListener);
            this.attachedSocket.off('disconnect', this.disconnectListener);
            this.attachedSocket = undefined;
        }
        this.resetTelemetry();
    }

    private computeDelta(previous: PoseData, current: PoseData): number {
        const dx = current.pose.position.x - previous.pose.position.x;
        const dy = current.pose.position.y - previous.pose.position.y;
        const dz = current.pose.position.z - previous.pose.position.z;
        const delta = Math.sqrt(dx * dx + dy * dy + dz * dz);
        return Number.isFinite(delta) ? delta : 0;
    }

    private quaternionToYaw(orientation: { x: number; y: number; z: number; w: number }): number {
        const { x, y, z, w } = orientation;
        return Math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z));
    }

    private normalizeRobotId(robotId: string): RobotId | null {
        if (robotId === 'limo1' || robotId === 'limo2') return robotId;
        return null;
    }
}
