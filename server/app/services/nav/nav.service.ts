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
    private goalHandles: Record<RobotId, rclnodejs.ClientGoalHandle<any> | null> = {
        limo1: null,
        limo2: null,
    };
    private isNavigating: Record<RobotId, boolean> = { limo1: false, limo2: false };
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
        try {
            await this.dispatchWaypoints(robot, waypoints);
        } catch (err) {
            this.logger.error(`Error sending waypoints for ${robot}`, (err as Error).stack);
        } finally {
            this.isNavigating[robot] = false;
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
        const goalHandle = await client.sendGoal(
            goal,
            (feedback) =>
                this.logger.debug(
                    `[${robot}] current waypoint index: ${feedback?.current_waypoint}, current pose: ${feedback?.current_pose?.pose}`,
                ),
        );
        this.goalHandles[robot] = goalHandle;
        try {
            const result = await goalHandle.getResult();
            this.logger.log(`[${robot}] waypoint result: ${result?.status}`);
        } finally {
            this.goalHandles[robot] = null;
        }
    }

    async cancelGoal(robot: RobotId) {
        if (!this.isInit) {
            this.logger.warn(`Attempted to cancel goal for ${robot} before NavService init`);
            return;
        }

        const goalHandle = this.goalHandles[robot];
        if (!goalHandle) {
            this.logger.warn(`No active goal to cancel for ${robot}`);
            return;
        }

        try {
            const cancelResponse = await goalHandle.cancelGoal();
            const cancelReturnCode = cancelResponse?.return_code ?? -1;
            this.logger.log(`[${robot}] cancel request completed with code ${cancelReturnCode}`);

            if (cancelReturnCode === 0) {
                this.socketService.sendStateToAllSockets(robot, 'NAV_GOAL_CANCELLED');
            } else {
                this.logger.warn(`[${robot}] cancel request rejected with code ${cancelReturnCode}`);
            }
        } catch (err) {
            this.logger.error(`[${robot}] Failed to cancel goal`, (err as Error).stack);
        } finally {
            this.goalHandles[robot] = null;
        }
    }

    private buildRosTime() {
        const now = Date.now();
        return {
        sec: Math.floor(now / 1000),
        nanosec: (now % 1000) * 1e6,
        };
    }

}
