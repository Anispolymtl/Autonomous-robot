import {
  WebSocketGateway,
  WebSocketServer,
  SubscribeMessage,
  MessageBody,
} from '@nestjs/websockets';
import { Server, Socket } from 'socket.io';
import { SocketService } from '@app/services/socket/socket.service';

@WebSocketGateway({ namespace: '/client' })
export class TelemetryGateway {
    @WebSocketServer() server: Server;

    constructor(
        private socketService: SocketService
    ) {}

    handleConnection(socket: Socket) {
        console.log(`Connexion par l'utilisateur avec id : ${socket.id}`);
        this.socketService.addSocket(socket);
    }

    handleDisconnect(socket: Socket) {
        console.log(`Déconnexion de l'utilisateur avec id : ${socket.id}`);
        this.socketService.removeSocket(socket);
    }

}