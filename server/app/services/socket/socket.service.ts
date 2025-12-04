import { OccupancyGrid } from "@common/interfaces/occupancy-grid";
import { Injectable } from "@nestjs/common";
import { Socket } from 'socket.io';

type RobotId = 'limo1' | 'limo2';
type Point2D = {x: number, y: number};

@Injectable()
export class SocketService{
    private sockets: Set<Socket> = new Set();
    private payloads: Record<RobotId, {poseData: any, mapData: any, points: Point2D[]}> = {
        limo1: {poseData: null, mapData: null, points: []},
        limo2: {poseData: null, mapData: null, points: []}
    };
    private mergedMap: any;
    private broadcastInterval?: NodeJS.Timeout;



    onModuleInit() {
        this.broadcastInterval = setInterval(() => this.broadcastSnapshots(), 1000);
    }

    onModuleDestroy() {
        if (this.broadcastInterval) clearInterval(this.broadcastInterval);
    }

    private broadcastSnapshots() {
        (['limo1', 'limo2'] as RobotId[]).forEach((robot) => {
            const { poseData, mapData, points } = this.payloads[robot];
            this.sockets.forEach((socket) => {
                if (mapData) socket.emit(`/${robot}/mapUpdate`, mapData);
                if (poseData) socket.emit(`/${robot}/poseUpdate`, poseData);
                if (points.length) socket.emit(`/${robot}/newPoints`, points);
            });
        });
    }

    addSocket(socket: Socket) {
        this.sockets.add(socket);
    }

    removeSocket(socket: Socket) {
        this.sockets.delete(socket);
    }

    sendStateToAllSockets(robot: RobotId, state: any) {
        this.sockets.forEach((socket) => {
            socket.emit('stateUpdate', {robot, state});
        });
    }

    sendWaypointStatus(robot: RobotId, status: 'started' | 'completed') {
        this.sockets.forEach((socket) => {
            socket.emit('waypointsStatus', { robot, status });
        });
    }

    sendMapToAllSockets(mapData: any, robot: RobotId) {
        this.payloads[robot].mapData = mapData;
        this.sockets.forEach((socket) => {
            socket.emit(`/${robot}/mapUpdate`, mapData);
        });
    }

    sendPoseToAllSockets(robot: RobotId, poseData: any) {
        this.payloads[robot].poseData = poseData;
        this.sockets.forEach((socket) => {
            socket.emit('poseUpdate', { robot, poseData });
        });
    }

    sendMergedMapToAllSockets(mapData?: any) {
        if (mapData) {
            this.mergedMap = mapData;
            this.sockets.forEach((socket) => {
                socket.emit('mergedMapUpdate', { map: mapData });
            });
        } else if (this.mergedMap) {
            this.sockets.forEach((socket) => {
                socket.emit('mergedMapUpdate', { map: this.mergedMap });
            });
        }
    }

    sendPointsToAllSockets(robot: RobotId, points: Point2D[]) {
        this.payloads[robot].points = points;
        this.sockets.forEach(socket => {
            socket.emit('newPoints', {robot, points});
        });
    }

    sendExplorationStepToAllSockets(msg: any, robot: RobotId) {
        console.log('step', msg)
        this.sockets.forEach(socket => {
            socket.emit('expStep', {robot, msg});
        });
    }

    sendMissedPoints(robot: RobotId, missedPoints: number[], originalWaypoints: Point2D[]) {
        this.sockets.forEach(socket => {
            socket.emit(`/${robot}/missedWaypoints`, {missedPoints, originalWaypoints});
        });
    }

    getMaps(): Record<RobotId, any> {
        const limo1Map = this.payloads.limo1.mapData;
        const limo2Map = this.payloads.limo2.mapData;
        return {
            limo1: limo1Map,
            limo2: limo2Map
        };
    }

    getMergedMap(): any {
        return this.mergedMap;
    }
}
