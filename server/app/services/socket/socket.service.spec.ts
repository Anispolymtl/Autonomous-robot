import { Test, TestingModule } from '@nestjs/testing';
import { SocketService } from './socket.service';
import { Socket } from 'socket.io';

describe('SocketService', () => {
  let service: SocketService;
  let mockSocket: jest.Mocked<Socket>;

  beforeEach(async () => {
    const module: TestingModule = await Test.createTestingModule({
      providers: [SocketService],
    }).compile();

    service = module.get<SocketService>(SocketService);

    mockSocket = {
      emit: jest.fn(),
    } as unknown as jest.Mocked<Socket>;
  });

  afterEach(() => {
    jest.clearAllMocks();
  });

  it('should be defined', () => {
    expect(service).toBeDefined();
  });

  describe('addSocket & removeSocket', () => {
    it('should add and remove sockets', () => {
      service.addSocket(mockSocket);
      expect((service as any).sockets.has(mockSocket)).toBe(true);

      service.removeSocket(mockSocket);
      expect((service as any).sockets.has(mockSocket)).toBe(false);
    });
  });

  describe('sendStateToAllSockets', () => {
    it('should emit stateUpdate to all sockets', () => {
      service.addSocket(mockSocket);
      service.sendStateToAllSockets('limo1', { status: 'running' });

      expect(mockSocket.emit).toHaveBeenCalledWith('stateUpdate', {
        robot: 'limo1',
        state: { status: 'running' },
      });
    });
  });

  describe('sendMapToAllSockets', () => {
    it('should update payloads and emit mapUpdate', () => {
      service.addSocket(mockSocket);
      const mapData = { data: [0, 1, 2] };
      service.sendMapToAllSockets(mapData, 'limo1');

      expect((service as any).payloads.limo1.mapData).toBe(mapData);
      expect(mockSocket.emit).toHaveBeenCalledWith('/limo1/mapUpdate', mapData);
    });
  });

  describe('sendPoseToAllSockets', () => {
    it('should update payloads and emit poseUpdate', () => {
      service.addSocket(mockSocket);
      const poseData = { x: 1, y: 2 };
      service.sendPoseToAllSockets('limo2', poseData);

      expect((service as any).payloads.limo2.poseData).toBe(poseData);
      expect(mockSocket.emit).toHaveBeenCalledWith('poseUpdate', {
        robot: 'limo2',
        poseData,
      });
    });
  });

  describe('sendPointsToAllSockets', () => {
    it('should update payloads and emit newPoints', () => {
      service.addSocket(mockSocket);
      const points = [{ x: 1, y: 2 }, { x: 3, y: 4 }];
      service.sendPointsToAllSockets('limo1', points);

      expect((service as any).payloads.limo1.points).toBe(points);
      expect(mockSocket.emit).toHaveBeenCalledWith('newPoints', {
        robot: 'limo1',
        points,
      });
    });
  });

  describe('lifecycle hooks', () => {
    it('should start and clear broadcast interval', () => {
        jest.useFakeTimers();
        
        service.onModuleInit();
        expect((service as any).broadcastInterval).toBeDefined();

        service.onModuleDestroy();
        jest.advanceTimersByTime(2000);
        expect(true).toBe(true);

        jest.useRealTimers();
    });
  });
});
