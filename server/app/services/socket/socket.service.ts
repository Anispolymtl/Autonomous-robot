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

    sendMapToAllSockets(mapData: any, robot: RobotId) {
        this.payloads[robot].mapData = mapData;
        this.sockets.forEach((socket) => {
            socket.emit(`/${robot}/mapUpdate`, mapData);
        });
    }

    sendPoseToAllSockets(robot: RobotId, poseData: any) {
        console.log(`Sending pose data for ${robot} to all sockets`);
        this.payloads[robot].poseData = poseData;
        this.sockets.forEach((socket) => {
            socket.emit('poseUpdate', { robot, poseData });
        });
    }

    sendPointsToAllSockets(robot: RobotId, points: Point2D[]) {
        console.log(`Sending new points for robot ${robot} to all sockets`);
        this.payloads[robot].points = points;
        console.log(this.payloads[robot].points);
        this.sockets.forEach(socket => {
            socket.emit('newPoints', {robot, points});
        });
    }
}
