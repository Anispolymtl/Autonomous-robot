import { Test, TestingModule } from '@nestjs/testing';
import { ClientGateway } from './client.gateway';
import { SocketService } from '@app/services/socket/socket.service';
import { MissionRuntimeService, MissionCreatePayload } from '@app/services/mission-runtime/mission-runtime.service';
import { NavService } from '@app/services/nav/nav.service';
import { Socket } from 'socket.io';


describe('ClientGateway', () => {
  let gateway: ClientGateway;
  let socketService: SocketService;
  let missionRuntimeService: MissionRuntimeService;
  let navService: NavService;
  let mockSocket: Partial<Socket>;

  beforeEach(async () => {
    const module: TestingModule = await Test.createTestingModule({
      providers: [
        ClientGateway,
        { provide: SocketService, useValue: { addSocket: jest.fn(), removeSocket: jest.fn() } },
        { provide: MissionRuntimeService, useValue: {
            createMission: jest.fn(),
            updateMission: jest.fn(),
            appendLog: jest.fn(),
            completeMission: jest.fn()
        } },
        { provide: NavService, useValue: {
            addPoint: jest.fn(),
            removePoint: jest.fn(),
            startGoal: jest.fn()
        } }
      ],
    }).compile();

    gateway = module.get<ClientGateway>(ClientGateway);
    socketService = module.get(SocketService);
    missionRuntimeService = module.get(MissionRuntimeService);
    navService = module.get(NavService);

    mockSocket = {
      id: 'socket1',
      emit: jest.fn()
    };
  });

  it('should be defined', () => {
    expect(gateway).toBeDefined();
  });

  it('should handle connection', () => {
    gateway.handleConnection(mockSocket as Socket);
    expect(socketService.addSocket).toHaveBeenCalledWith(mockSocket);
  });

  it('should handle disconnection', () => {
    gateway.handleDisconnect(mockSocket as Socket);
    expect(socketService.removeSocket).toHaveBeenCalledWith(mockSocket);
  });

  it('should call navService.addPoint on "point" message', () => {
    const payload = { robot: 'limo1' as const, point: { x: 1, y: 2 } };
    gateway.onPointGet(mockSocket as Socket, payload);
    expect(navService.addPoint).toHaveBeenCalledWith(payload);
  });

  it('should call navService.removePoint on "removePoint" message', () => {
    const payload = { robot: 'limo1' as const, index: 0 };
    gateway.onPointRemove(mockSocket as Socket, payload);
    expect(navService.removePoint).toHaveBeenCalledWith(payload);
  });

  it('should call navService.startGoal on "startNavGoal" message', () => {
    const payload = { robot: 'limo1' as const };
    gateway.onGoalGet(mockSocket as Socket, payload);
    expect(navService.startGoal).toHaveBeenCalledWith(payload.robot);
  });

  it('should handle mission creation error', () => {
    const payload: MissionCreatePayload = { missionName: 'TestMission', robots: ['limo1'], mode: 'SIMULATION', distance: 10, durationSec: 100 };
    (missionRuntimeService.createMission as jest.Mock).mockImplementation(() => { throw new Error('Failed'); });

    gateway.handleMissionCreate(mockSocket as Socket, payload);

    expect(mockSocket.emit).toHaveBeenCalledWith('mission:error', { message: 'Failed' });
  });

  it('should handle mission update error', () => {
    const payload = { missionId: 'm1', data: { missionName: 'Updated' } };
    (missionRuntimeService.updateMission as jest.Mock).mockImplementation(() => { throw new Error('UpdateFailed'); });

    gateway.handleMissionUpdate(mockSocket as Socket, payload);
    expect(mockSocket.emit).toHaveBeenCalledWith('mission:error', { message: 'UpdateFailed', missionId: 'm1' });
  });

  it('should handle mission completion error', () => {
    const payload = { missionId: 'm1' };
    (missionRuntimeService.completeMission as jest.Mock).mockImplementation(() => { throw new Error('CompleteFailed'); });

    gateway.handleMissionComplete(mockSocket as Socket, payload);

    expect(mockSocket.emit).toHaveBeenCalledWith('mission:error', { message: 'CompleteFailed', missionId: 'm1' });
  });
});
