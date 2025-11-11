import { Test, TestingModule } from '@nestjs/testing';
import { ClientGateway } from './client.gateway';
import { SocketService } from '@app/services/socket/socket.service';
import { MissionRuntimeService, MissionCreatePayload } from '@app/services/mission-runtime/mission-runtime.service';
import { NavService } from '@app/services/nav/nav.service';
import { Socket } from 'socket.io';

describe('ClientGateway', () => {
  let gateway: ClientGateway;
  let socketService: Partial<SocketService>;
  let missionRuntimeService: Partial<MissionRuntimeService>;
  let navService: Partial<NavService>;
  let mockSocket: Partial<Socket>;

  beforeEach(async () => {
    socketService = {
      addSocket: jest.fn(),
      removeSocket: jest.fn(),
    };

    missionRuntimeService = {
      createMission: jest.fn().mockImplementation((socketId: string, payload: MissionCreatePayload) => ({
        missionId: 'mission1',
        ...payload,
      })),
      updateMission: jest.fn().mockImplementation((missionId: string, data: any) => ({
        missionId,
        ...data,
      })),
      appendLog: jest.fn().mockImplementation((missionId: string, log: any) => ({
        missionId,
        log,
      })),
      completeMission: jest.fn().mockImplementation((missionId: string) => ({
        missionId,
        status: 'completed',
      })),
    };

    navService = {
      addPoint: jest.fn(),
      removePoint: jest.fn(),
      startGoal: jest.fn(),
    };

    mockSocket = {
      id: 'socket1',
      emit: jest.fn(),
    };

    const module: TestingModule = await Test.createTestingModule({
      providers: [
        ClientGateway,
        { provide: SocketService, useValue: socketService },
        { provide: MissionRuntimeService, useValue: missionRuntimeService },
        { provide: NavService, useValue: navService },
      ],
    }).compile();

    gateway = module.get<ClientGateway>(ClientGateway);
  });

  it('should be defined', () => {
    expect(gateway).toBeDefined();
  });

  describe('handleConnection', () => {
    it('should add socket', () => {
      gateway.handleConnection(mockSocket as Socket);
      expect(socketService.addSocket).toHaveBeenCalledWith(mockSocket);
    });
  });

  describe('handleDisconnect', () => {
    it('should remove socket', () => {
      gateway.handleDisconnect(mockSocket as Socket);
      expect(socketService.removeSocket).toHaveBeenCalledWith(mockSocket);
    });
  });

  describe('onPointGet', () => {
    it('should call navService.addPoint', () => {
      const payload = { robot: 'limo1' as 'limo1', point: { x: 1, y: 2 } };
      gateway.onPointGet(mockSocket as Socket, payload);
      expect(navService.addPoint).toHaveBeenCalledWith(payload);
    });
  });

  describe('onPointRemove', () => {
    it('should call navService.removePoint', () => {
      const payload = { robot: 'limo2' as 'limo2', index: 0 };
      gateway.onPointRemove(mockSocket as Socket, payload);
      expect(navService.removePoint).toHaveBeenCalledWith(payload);
    });
  });

  describe('onGoalGet', () => {
    it('should call navService.startGoal', () => {
      const payload = { robot: 'limo1' as 'limo1' };
      gateway.onGoalGet(mockSocket as Socket, payload);
      expect(navService.startGoal).toHaveBeenCalledWith('limo1');
    });
  });

  describe('handleMissionCreate', () => {
    it('should create mission and emit', () => {
      const payload: MissionCreatePayload = { name: 'Test Mission' } as any;
      gateway.handleMissionCreate(mockSocket as Socket, payload);
      expect(missionRuntimeService.createMission).toHaveBeenCalledWith(mockSocket.id, payload);
      expect(mockSocket.emit).toHaveBeenCalledWith('mission:created', expect.objectContaining({
        missionId: 'mission1',
      }));
    });
  });

  describe('handleMissionUpdate', () => {
    it('should update mission and emit', () => {
      const payload = { missionId: 'mission1', data: { distance: 100 } };
      gateway.handleMissionUpdate(mockSocket as Socket, payload);
      expect(missionRuntimeService.updateMission).toHaveBeenCalledWith(payload.missionId, payload.data);
      expect(mockSocket.emit).toHaveBeenCalledWith('mission:updated', expect.objectContaining({
        missionId: 'mission1',
      }));
    });
  });

  describe('handleMissionLog', () => {
    it('should append log and emit', () => {
      const payload = { missionId: 'mission1', log: { message: 'Log entry' } };
      gateway.handleMissionLog(mockSocket as Socket, payload);
      expect(missionRuntimeService.appendLog).toHaveBeenCalledWith(payload.missionId, payload.log);
      expect(mockSocket.emit).toHaveBeenCalledWith('mission:updated', expect.objectContaining({
        missionId: 'mission1',
      }));
    });
  });

  describe('handleMissionComplete', () => {
    it('should complete mission and emit', () => {
      const payload = { missionId: 'mission1' };
      gateway.handleMissionComplete(mockSocket as Socket, payload);
      expect(missionRuntimeService.completeMission).toHaveBeenCalledWith(payload.missionId);
      expect(mockSocket.emit).toHaveBeenCalledWith('mission:finalized', expect.objectContaining({
        missionId: 'mission1',
      }));
    });
  });
});
