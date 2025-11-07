import { Injectable } from "@nestjs/common";
import { Socket } from 'socket.io';

type RobotId = 'limo1' | 'limo2';

@Injectable()
export class SocketService{
    private sockets: Set<Socket> = new Set();

    addSocket(socket: Socket) {
        this.sockets.add(socket);
    }

    removeSocket(socket: Socket) {
        this.sockets.delete(socket);
    }
    
    sendMapToAllSockets(mapData: any) {
        console.log('Sending map data to all sockets');
        this.sockets.forEach((socket) => {
            socket.emit('mapUpdate', mapData);
        });
    }

    sendPoseToAllSockets(robot: RobotId, poseData: any) {
        console.log(`Sending pose data for ${robot} to all sockets`);
        this.sockets.forEach((socket) => {
            socket.emit('poseUpdate', { robot, pose: poseData });
        });
    }
}
