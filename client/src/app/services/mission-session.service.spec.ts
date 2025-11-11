import { TestBed } from '@angular/core/testing';
import { MissionSessionService } from './mission-session.service';
import { SocketService } from './socket.service';
import { MissionModeService } from './mission-mode.service';

describe('MissionSessionService', () => {
  let service: MissionSessionService;
  let socketService: jasmine.SpyObj<SocketService>;
  let missionModeService: jasmine.SpyObj<MissionModeService>;

  beforeEach(() => {
    const socketSpy = jasmine.createSpyObj('SocketService', ['send', 'on', 'off', 'connect', 'isSocketAlive', 'disconnect']);
    const modeSpy = jasmine.createSpyObj('MissionModeService', ['setMode', 'fetchActiveMission']);

    TestBed.configureTestingModule({
      providers: [
        MissionSessionService,
        { provide: SocketService, useValue: socketSpy },
        { provide: MissionModeService, useValue: modeSpy },
      ],
    });

    service = TestBed.inject(MissionSessionService);
    socketService = TestBed.inject(SocketService) as jasmine.SpyObj<SocketService>;
    missionModeService = TestBed.inject(MissionModeService) as jasmine.SpyObj<MissionModeService>;
  });

  it('should be created', () => {
    expect(service).toBeTruthy();
  });

  describe('initializeMission', () => {
    it('should send mission:create and resolve on mission:created', async () => {
      const mockMission = {
        missionName: 'Test',
        robots: ['limo1', 'limo2'],
        mode: 'SIMULATION' as 'SIMULATION' | 'REAL',
        distance: 0,
        durationSec: 0,
        logs: [],
        status: 'PENDING',
        _id: '1',
        createdAt: new Date(),
      };

      socketService.on.and.callFake((event, cb) => {
        if (event === 'mission:created') cb({ missionId: '123', mission: mockMission } as any);
      });
      socketService.isSocketAlive.and.returnValue(true);

      const result = await service.initializeMission('Test', 'SIMULATION');
      expect(result.missionId).toBe('123');
      expect(result.mission).toEqual(mockMission);
      expect(missionModeService.setMode).toHaveBeenCalledWith('SIMULATION');
      expect(socketService.send).toHaveBeenCalledWith('mission:create', jasmine.any(Object));
    });

    it('should reject on mission:error', async () => {
      socketService.on.and.callFake((event, cb) => {
        if (event === 'mission:error') cb({ message: 'Error!' } as any);
      });
      socketService.isSocketAlive.and.returnValue(true);

      try {
        await service.initializeMission('Test', 'SIMULATION');
        fail('Expected initializeMission to throw');
      } catch (err: any) {
        expect(err.message).toBe('Error!');
      }
    });
  });

  describe('markMissionStarted', () => {
    it('should set timestamp and send mission update', () => {
      (service as any).missionId = '123';
      spyOn(Date, 'now').and.returnValue(1000);

      service.markMissionStarted();

      expect((service as any).missionStartTimestamp).toBe(1000);
      expect(socketService.send).toHaveBeenCalledWith('mission:update', jasmine.objectContaining({
        missionId: '123',
        data: jasmine.objectContaining({ status: 'RUNNING', durationSec: 0 }),
      }));
    });
  });

  describe('rehydrateActiveMission', () => {
    it('should do nothing if mission already active', async () => {
      (service as any).missionId = '123';
      await service.rehydrateActiveMission();
      expect(missionModeService.fetchActiveMission).not.toHaveBeenCalled();
    });
  });

  describe('appendLog', () => {
    it('should send log if mission active', () => {
      (service as any).missionId = '123';
      const logEntry = { message: 'Hello' };
      service.appendLog(logEntry);

      expect(socketService.send).toHaveBeenCalledWith('mission:add-log', jasmine.objectContaining({
        missionId: '123',
        log: jasmine.objectContaining({ message: 'Hello' }),
      }));
    });

    it('should not send log if no mission active', () => {
      service.appendLog({ message: 'Hello' });
      expect(socketService.send).not.toHaveBeenCalled();
    });
  });

  describe('completeMission', () => {
    it('should return null if no mission active', async () => {
      const result = await service.completeMission();
      expect(result).toBeNull();
    });
  });
});
