import { Injectable, Logger } from "@nestjs/common";
import { RosService } from "../ros/ros.service";
import { SocketService } from "../socket/socket.service";

type RobotId = 'limo1' | 'limo2';

@Injectable()
export class BatteryService {
    private readonly logger = new Logger(BatteryService.name);
    private readonly tickMs = 6700;
    private readonly drainPerTick = 1;
    private countdownInterval?: NodeJS.Timeout;
    private batteryLevels: Record<RobotId, number> = { limo1: 100, limo2: 100 };
    private lowBatteryNotified: Record<RobotId, boolean> = { limo1: false, limo2: false };

    constructor(
        private rosService: RosService,
        private socketService: SocketService
    ) {}

    /**
     * Starts the battery countdown for both robots if not already running.
     */
    startBattery() {
        if (this.countdownInterval) return;
        this.socketService.sendBatteryToAllSockets(this.batteryLevels)
        this.countdownInterval = setInterval(() => {
            (['limo1', 'limo2'] as RobotId[]).forEach((robot) => {
                if (this.batteryLevels[robot] <= 0) return;

                this.batteryLevels[robot] = Math.max(0, this.batteryLevels[robot] - this.drainPerTick);
                this.socketService.sendBatteryToAllSockets(this.batteryLevels)
                if (this.batteryLevels[robot] <= 30 && !this.lowBatteryNotified[robot]) {
                    this.handleLowBattery(robot);
                }
            });
        }, this.tickMs);
    }

    /**
     * Resets the battery level of a specific robot to full charge.
     */
    resetBattery(robot: RobotId) {
        this.batteryLevels[robot] = 100;
        this.lowBatteryNotified[robot] = false;
    }

    /**
     * Stops the countdown and resets all battery levels and notifications.
     */
    endBattery() {
        if (this.countdownInterval) {
            clearInterval(this.countdownInterval);
            this.countdownInterval = undefined;
        }
        this.batteryLevels = { limo1: 100, limo2: 100 };
        this.lowBatteryNotified = { limo1: false, limo2: false };
    }

    /**
     * Hook executed when a robot reaches the low battery threshold.
     */
    private handleLowBattery(robot: RobotId) {
        this.logger.warn(`[${robot}] Battery reached 30%`);
        this.lowBatteryNotified[robot] = true;
        this.rosService.returnToBaseIndividual(robot);
    }
}
