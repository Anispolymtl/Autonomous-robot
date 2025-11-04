import { Injectable } from "@nestjs/common";
import { Socket } from 'socket.io';

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
        this.sockets.forEach((socket) => {
            socket.emit('mapUpdate', mapData);
        });
    }
}