import { Injectable, Logger } from "@nestjs/common";
import * as rclnodejs from 'rclnodejs';
import { SocketService } from "../socket/socket.service";

type RobotId = 'limo1' | 'limo2';
type Point2D = { x: number; y: number };

@Injectable()
export class NavService {
    private readonly logger = new Logger(NavService.name);
    private navClient1: any;
    private navClient2: any;
    private stateClient1: any;
    private stateClient2: any;
    private readonly setStateService: any;
    private readonly robotStateConstants: any;
    private points: Record<RobotId, Point2D[]> = { limo1: [], limo2: [] };
    private isNavigating: Record<RobotId, boolean> = { limo1: false, limo2: false };
    private goalHandles: Record<RobotId, any> = { limo1: null, limo2: null };
    private returnInProgress: Record<RobotId, boolean> = { limo1: false, limo2: false };
    private isInit: boolean = false;
    
    constructor(
        private socketService: SocketService
    ) {
        // Charger les types/services ROS2 (fallback si non disponibles)
        let constants;
        try {
            const pkg: any = (rclnodejs.require as any)?.('limo_interfaces');
            constants = pkg?.msg?.RobotState;
            this.setStateService = pkg?.srv?.SetRobotState;
        } catch (err) {
            this.logger.warn('[NAV] Impossible de charger limo_interfaces, utilisation valeurs par defaut', err as Error);
        }

        this.robotStateConstants = constants ?? {
            WAIT: 0,
            EXPLORATION: 1,
            NAVIGATION: 2,
            RETURN_TO_BASE: 3,
            CUSTOM_MISSION: 4,
        };
    }

    initNavService(nodeLimo1: rclnodejs.Node, nodeLimo2: rclnodejs.Node) {
        this.navClient1 = new rclnodejs.ActionClient(nodeLimo1, 'nav2_msgs/action/FollowWaypoints', 'follow_waypoints');
        this.navClient2 = new rclnodejs.ActionClient(nodeLimo2, 'nav2_msgs/action/FollowWaypoints', 'follow_waypoints');
        if (this.setStateService) {
            this.stateClient1 = nodeLimo1.createClient(this.setStateService, 'set_state');
            this.stateClient2 = nodeLimo2.createClient(this.setStateService, 'set_state');
        }
        this.isInit = true;
    }

    addPoint(payload: {robot: RobotId; point: Point2D}) {
        if (!this.isInit || this.isNavigating[payload.robot]) return [];
        this.points[payload.robot].push(payload.point);
        this.socketService.sendPointsToAllSockets(payload.robot, this.points[payload.robot]);
    }

    removePoint(payload: {robot: RobotId, index: number}) {
        if (!this.isInit || payload.index >= this.points[payload.robot].length || this.isNavigating[payload.robot] || payload.index < 0) return [];
        this.points[payload.robot].splice(payload.index, 1);
        this.socketService.sendPointsToAllSockets(payload.robot, this.points[payload.robot]);
    }

    async startGoal(robot: RobotId) {
        if (!this.isInit || this.isNavigating[robot]) return;

        const queue = [...this.points[robot]];
        if (!queue.length) return;

        this.isNavigating[robot] = true;

        try {
            const navReady = await this.ensureNavigationState(robot);
            if (!navReady) {
                this.logger.warn(`[${robot}] Navigation annulée: impossible de passer en état NAVIGATION`);
                return;
            }

            // On purge la liste envoyée au robot
            this.points[robot] = [];
            this.socketService.sendPointsToAllSockets(robot, this.points[robot]);

            await this.processPointQueue(robot, queue);
        } catch (err) {
            this.logger.error(`Failed to process queue for ${robot}`, (err as Error).stack);
        } finally {
            this.isNavigating[robot] = false;
        }
    }

    private async processPointQueue(robot: RobotId, queue: Point2D[]) {
        const waypoints = queue.splice(0, queue.length);
        console.log(waypoints);
        try {
            await this.dispatchWaypoints(robot, waypoints);
        } catch (err) {
            this.logger.error(`Error sending waypoints for ${robot}`, (err as Error).stack);
        }
        // La remise à l'état WAIT est maintenant gérée côté ROS (StateManager/Mission).
    }

    private async dispatchWaypoints(robot: RobotId, waypoints: Point2D[]) {
        const client = robot === 'limo1' ? this.navClient1 : this.navClient2;
        if (!client) throw new Error(`Action client for ${robot} not ready`);

        await (client as any).waitForServer?.();
        const goal = {
        poses: waypoints.map((point) => ({
            header: {
            stamp: this.buildRosTime(),
            frame_id: `${robot}/map`,
            },
            pose: {
            position: { x: point.x, y: point.y, z: 0 },
            orientation: { x: 0, y: 0, z: 0, w: 1 },
            },
        })),
        behavior_tree: '',
        };
        console.log(goal);
        console.log(goal.poses)
        const goalHandle = await client.sendGoal(goal, (feedback) =>
        this.logger.debug(
            `[${robot}] current waypoint index: ${feedback?.current_waypoint}, current pose: ${feedback?.current_pose?.pose}`,
        ),
        );
        this.goalHandles[robot] = goalHandle;
        const result = await goalHandle.getResult();
        this.logger.log(`[${robot}] waypoint result: `, result);

        // rclnodejs returns missed_waypoints as an Int32Array, which stringifies like { "0": 1 }.
        const missed = this.normalizeMissedWaypoints((result as any)?.result?.missed_waypoints ?? (result as any)?.missed_waypoints);
        if (missed.length > 0) {
            console.log(missed);
            this.handleMissedPoints(missed, waypoints, robot);
        }
        this.goalHandles[robot] = null;
        return result?.status;
    }

    private normalizeMissedWaypoints(raw: unknown): number[] {
        if (!raw) return [];

        // Simple fallback: expect plain objects like { "0": 1, "1": 3 }
        if (typeof raw === 'object') {
            const obj = raw as Record<string, unknown>;
            return Object.keys(obj)
                .filter((key) => /^\d+$/.test(key))
                .sort((a, b) => Number(a) - Number(b))
                .map((key) => Number(obj[key]))
                .filter((n) => Number.isFinite(n));
        }

        return [];
    }

    private handleMissedPoints(missedPoints: number[], originalWaypoints: Point2D[], robot: RobotId) {
        this.logger.warn(`[${robot}] Missed waypoints indices: ${missedPoints.join(', ')}`);
        this.socketService.sendMissedPoints(robot, missedPoints, originalWaypoints);
    }

    /**
     * Annule la navigation en cours pour un robot et réinitialise l'état.
     */
    async cancelNavigation(robot: RobotId) {
        const handle = this.goalHandles[robot];
        if (!handle) {
            this.isNavigating[robot] = false;
            return;
        }

        try {
            await handle.cancelGoal?.();
            this.logger.log(`[${robot}] Navigation annulée (retour à la base demandé)`);
        } catch (err) {
            this.logger.warn(`[${robot}] Échec annulation navigation: ${(err as Error).message}`);
        } finally {
            this.goalHandles[robot] = null;
            this.isNavigating[robot] = false;
            this.socketService.sendWaypointStatus(robot, 'completed');
            // Ne pas écraser l'état "Retour à la base" ici : le caller gère l'état affiché
        }
    }

    setReturnInProgress(robot: RobotId, inProgress: boolean) {
        this.returnInProgress[robot] = inProgress;
    }

    isReturnInProgress(robot: RobotId): boolean {
        return !!this.returnInProgress[robot];
    }

    private async ensureNavigationState(robot: RobotId): Promise<boolean> {
        const ok = await this.setRobotState(robot, this.robotStateConstants.NAVIGATION);
        return ok;
    }

    private async setRobotState(robot: RobotId, state: number): Promise<boolean> {
        const client = robot === 'limo1' ? this.stateClient1 : this.stateClient2;
        if (!client) {
            this.logger.warn(`[${robot}] Client set_state non disponible`);
            return false;
        }

        // Attendre le service si possible
        if (typeof client.waitForService === 'function') {
            try {
                await client.waitForService(1000);
            } catch (err) {
                this.logger.warn(`[${robot}] Service set_state indisponible`, err as Error);
                return false;
            }
        }

        return new Promise<boolean>((resolve) => {
            const request = new (this.setStateService as any).Request();
            request.state = state;
            client.sendRequest(request, (response: { success?: boolean; message?: string }) => {
                if (response?.success) {
                    this.logger.log(`[${robot}] État -> ${state}`);
                    resolve(true);
                } else {
                    this.logger.warn(`[${robot}] Échec set_state: ${response?.message ?? 'réponse vide'}`);
                    resolve(false);
                }
            });
        });
    }

    private buildRosTime() {
        const now = Date.now();
        return {
        sec: Math.floor(now / 1000),
        nanosec: (now % 1000) * 1e6,
        };
    }

}
