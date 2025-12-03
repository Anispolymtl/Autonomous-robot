import { Injectable, Logger } from "@nestjs/common";

type RobotId = 'limo1' | 'limo2';

@Injectable()
export class BatteryService {
    private readonly logger = new Logger(BatteryService.name);
    private readonly tickMs = 10000;
    private readonly drainPerTick = 1;
    private countdownInterval?: NodeJS.Timeout;
    private batteryLevels: Record<RobotId, number> = { limo1: 100, limo2: 100 };
    private lowBatteryNotified: Record<RobotId, boolean> = { limo1: false, limo2: false };

    /**
     * Starts the battery countdown for both robots if not already running.
     */
    startBattery() {
        if (this.countdownInterval) return;

        this.countdownInterval = setInterval(() => {
            (['limo1', 'limo2'] as RobotId[]).forEach((robot) => {
                if (this.batteryLevels[robot] <= 0) return;

                this.batteryLevels[robot] = Math.max(0, this.batteryLevels[robot] - this.drainPerTick);
                if (this.batteryLevels[robot] <= 30 && !this.lowBatteryNotified[robot]) {
                    this.lowBatteryNotified[robot] = true;
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
        // TODO: add low battery handling (e.g., notify UI, trigger return to base)
    }
}
