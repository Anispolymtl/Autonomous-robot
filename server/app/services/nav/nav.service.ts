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
    private points: Record<RobotId, Point2D[]> = { limo1: [], limo2: [] };
    private isNavigating: Record<RobotId, boolean> = { limo1: false, limo2: false };
    private goalHandles: Record<RobotId, any> = { limo1: null, limo2: null };
    private returnInProgress: Record<RobotId, boolean> = { limo1: false, limo2: false };
    private isInit: boolean = false;
    
    constructor(
        private socketService: SocketService
    ) {}

    initNavService(nodeLimo1: rclnodejs.Node, nodeLimo2: rclnodejs.Node) {
        this.navClient1 = new rclnodejs.ActionClient(nodeLimo1, 'nav2_msgs/action/FollowWaypoints', 'follow_waypoints');
        this.navClient2 = new rclnodejs.ActionClient(nodeLimo2, 'nav2_msgs/action/FollowWaypoints', 'follow_waypoints');
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

    startGoal(robot: RobotId) {
        const points = this.points[robot];
        this.points[robot] = [];
        if (!points.length || !this.isInit) return;
        if (!this.isNavigating[robot]) {
            this.socketService.sendStateToAllSockets(robot, 'Trajet en cours');
            this.socketService.sendWaypointStatus(robot, 'started');
            this.processPointQueue(robot, points).catch((err) =>
                this.logger.error(`Failed to process queue for ${robot}`, err.stack),
            );
        }
        this.socketService.sendPointsToAllSockets(robot, this.points[robot]);
    }

    private async processPointQueue(robot: RobotId, queue: Point2D[]) {
        this.isNavigating[robot] = true;
        const waypoints = queue.splice(0, queue.length);
        console.log(waypoints);
        let resultStatus: number | undefined;
        try {
            resultStatus = await this.dispatchWaypoints(robot, waypoints);
        } catch (err) {
            this.logger.error(`Error sending waypoints for ${robot}`, (err as Error).stack);
        } finally {
            // Toujours nettoyer le handle pour éviter un blocage sur les prochaines navigations
            this.goalHandles[robot] = null;
            this.isNavigating[robot] = false;
            this.socketService.sendWaypointStatus(robot, 'completed');

            // Si un retour à la base est en cours, ne pas écraser l'état affiché
            if (!this.returnInProgress[robot]) {
                if (resultStatus === 4 /* SUCCEEDED */) {
                    this.socketService.sendStateToAllSockets(robot, 'En attente');
                } else {
                    this.socketService.sendStateToAllSockets(robot, 'En attente (navigation interrompue)');
                }
            }
        }
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
        this.logger.log(`[${robot}] waypoint result: ${result?.status}`);
        this.goalHandles[robot] = null;
        return result?.status;
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

    private buildRosTime() {
        const now = Date.now();
        return {
        sec: Math.floor(now / 1000),
        nanosec: (now % 1000) * 1e6,
        };
    }

}
