import {
    WebSocketGateway,
    WebSocketServer,
    SubscribeMessage,
    MessageBody,
    ConnectedSocket,
} from '@nestjs/websockets';
import { Server, Socket } from 'socket.io';
import { SocketService } from '@app/services/socket/socket.service';
import { MissionCreatePayload, MissionRuntimeService } from '@app/services/mission-runtime/mission-runtime.service';
import { MissionLogEntry } from '@common/interfaces/mission-log-entry';
import { Mission } from '@app/model/database/mission';
import { NavService } from '@app/services/nav/nav.service';
import { RosService } from '@app/services/ros/ros.service';

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
        private missionRuntimeService: MissionRuntimeService,
        private navService: NavService,
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

    @SubscribeMessage('point')
    onPointGet(socket: Socket, payload: {robot: RobotId , point: Point2D}) {
        this.navService.addPoint(payload);
    }

    @SubscribeMessage('removePoint')
    onPointRemove(socket: Socket, payload: {robot: RobotId, index: number}) {
        this.navService.removePoint(payload);
    }

    @SubscribeMessage('startNavGoal')
    onGoalGet(socket: Socket, payload: {robot: RobotId}) {
        this.navService.startGoal(payload.robot);
    }

    @SubscribeMessage('nav:return-to-base')
    onReturnRequest(socket: Socket) {
        this.rosService.returnToBase();
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
            const mission = this.missionRuntimeService.updateMission(payload.missionId, payload.data ?? {});
            socket.emit('mission:updated', { missionId: mission.missionId, mission });
        } catch (error) {
            socket.emit('mission:error', { message: (error as Error).message, missionId: payload?.missionId });
        }
    }

    @SubscribeMessage('mission:add-log')
    handleMissionLog(@ConnectedSocket() socket: Socket, @MessageBody() payload: MissionLogPayload) {
        try {
            const mission = this.missionRuntimeService.appendLog(payload.missionId, payload.log);
            socket.emit('mission:updated', { missionId: mission.missionId, mission });
        } catch (error) {
            socket.emit('mission:error', { message: (error as Error).message, missionId: payload?.missionId });
        }
    }

    @SubscribeMessage('mission:complete')
    handleMissionComplete(@ConnectedSocket() socket: Socket, @MessageBody() payload: MissionCompletePayload) {
        try {
            const mission = this.missionRuntimeService.completeMission(payload.missionId);
            socket.emit('mission:finalized', { missionId: mission.missionId, mission });
        } catch (error) {
            socket.emit('mission:error', { message: (error as Error).message, missionId: payload?.missionId });
        }
    }
}
