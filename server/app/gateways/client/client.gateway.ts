import {
    WebSocketGateway,
    WebSocketServer,
    SubscribeMessage,
    MessageBody,
    ConnectedSocket,
} from '@nestjs/websockets';
import { Server, Socket } from 'socket.io';
import { SocketService } from '@app/services/socket/socket.service';
import { RosService } from '@app/services/ros.service';
import {
    MissionCreatePayload,
    MissionLogEntry,
    MissionRuntimeService,
} from '@app/services/mission-runtime/mission-runtime.service';
import { Mission } from '@app/model/database/mission';

type RobotId = 'limo1' | 'limo2';
type Point2D = { x: number; y: number };

interface MissionUpdatePayload {
    missionId: string;
    data: Partial<Mission>;
}

interface MissionLogPayload {
    missionId: string;
    log: MissionLogEntry;
}

interface MissionCompletePayload {
    missionId: string;
}

@WebSocketGateway({ namespace: '/client' })
export class ClientGateway {
    @WebSocketServer() server: Server;

    constructor(
        private socketService: SocketService,
        private rosService: RosService,
        private missionRuntimeService: MissionRuntimeService,
    ) {}

    handleConnection(socket: Socket) {
        console.log(`Connexion par l'utilisateur avec id : ${socket.id}`);
        this.socketService.addSocket(socket);
    }

    handleDisconnect(socket: Socket) {
        console.log(`Déconnexion de l'utilisateur avec id : ${socket.id}`);
        this.socketService.removeSocket(socket);
        this.missionRuntimeService.clearMissionsForSocket(socket.id);
    }

    @SubscribeMessage('pointlist')
    onPointListGet(@ConnectedSocket() socket: Socket, @MessageBody() payload: { robot: RobotId; points: Point2D[] }) {
        console.log('Réception des points depuis', socket.id);
        this.rosService.handlePoints(payload);
    }

    @SubscribeMessage('mission:create')
    handleMissionCreate(@ConnectedSocket() socket: Socket, @MessageBody() payload: MissionCreatePayload) {
        try {
            const mission = this.missionRuntimeService.createMission(socket.id, payload);
            socket.emit('mission:created', { missionId: mission.missionId, mission });
        } catch (error) {
            socket.emit('mission:error', { message: (error as Error).message });
        }
    }

    @SubscribeMessage('mission:update')
    handleMissionUpdate(@ConnectedSocket() socket: Socket, @MessageBody() payload: MissionUpdatePayload) {
        try {
            const mission = this.missionRuntimeService.updateMission(socket.id, payload.missionId, payload.data ?? {});
            socket.emit('mission:updated', { missionId: mission.missionId, mission });
        } catch (error) {
            socket.emit('mission:error', { message: (error as Error).message, missionId: payload?.missionId });
        }
    }

    @SubscribeMessage('mission:add-log')
    handleMissionLog(@ConnectedSocket() socket: Socket, @MessageBody() payload: MissionLogPayload) {
        try {
            const mission = this.missionRuntimeService.appendLog(socket.id, payload.missionId, payload.log);
            socket.emit('mission:updated', { missionId: mission.missionId, mission });
        } catch (error) {
            socket.emit('mission:error', { message: (error as Error).message, missionId: payload?.missionId });
        }
    }

    @SubscribeMessage('mission:complete')
    handleMissionComplete(@ConnectedSocket() socket: Socket, @MessageBody() payload: MissionCompletePayload) {
        try {
            const mission = this.missionRuntimeService.completeMission(socket.id, payload.missionId);
            socket.emit('mission:finalized', { missionId: mission.missionId, mission });
        } catch (error) {
            socket.emit('mission:error', { message: (error as Error).message, missionId: payload?.missionId });
        }
    }
}
