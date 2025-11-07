import {
  WebSocketGateway,
  WebSocketServer,
  SubscribeMessage,
  MessageBody,
} from '@nestjs/websockets';
import { Server, Socket } from 'socket.io';
import { SocketService } from '@app/services/socket/socket.service';
import { RosService } from '@app/services/ros.service';

type RobotId = 'limo1' | 'limo2';
type Point2D = { x: number; y: number };

@WebSocketGateway({ namespace: '/client' })
export class ClientGateway {
    @WebSocketServer() server: Server;

    constructor(
        private socketService: SocketService,
        private rosService: RosService
    ) {}

    handleConnection(socket: Socket) {
        console.log(`Connexion par l'utilisateur avec id : ${socket.id}`);
        this.socketService.addSocket(socket);
    }

    handleDisconnect(socket: Socket) {
        console.log(`Déconnexion de l'utilisateur avec id : ${socket.id}`);
        this.socketService.removeSocket(socket);
    }

    @SubscribeMessage('pointlist')
    onPointListGet(socket: Socket, payload: {robot: RobotId , points: Point2D[]}) {
        console.log('hola');
        this.rosService.handlePoints(payload);
    }


}