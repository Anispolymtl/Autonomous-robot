import {
  WebSocketGateway,
  WebSocketServer,
  SubscribeMessage,
  MessageBody,
} from '@nestjs/websockets';
import { Server, Socket } from 'socket.io';
import { SocketService } from '@app/services/socket/socket.service';
import { RosService } from '@app/services/ros.service';
import { NavService } from '@app/services/nav/nav.service';

type RobotId = 'limo1' | 'limo2';
type Point2D = { x: number; y: number };

@WebSocketGateway({ namespace: '/client' })
export class ClientGateway {
    @WebSocketServer() server: Server;

    constructor(
        private socketService: SocketService,
        private rosService: RosService,
        private navService: NavService
    ) {}

    handleConnection(socket: Socket) {
        console.log(`Connexion par l'utilisateur avec id : ${socket.id}`);
        this.socketService.addSocket(socket);
    }

    handleDisconnect(socket: Socket) {
        console.log(`Déconnexion de l'utilisateur avec id : ${socket.id}`);
        this.socketService.removeSocket(socket);
    }

    @SubscribeMessage('point')
    onPointListGet(socket: Socket, payload: {robot: RobotId , point: Point2D}) {
        const points = this.navService.addPoint(payload);
        this.server.emit('newPoints', {robot: payload.robot, points});
    }

    @SubscribeMessage('startNavGoal')
    onGoalGet(socket: Socket, payload: {robot: RobotId}) {
        this.navService.startGoal(payload.robot);
    }

}