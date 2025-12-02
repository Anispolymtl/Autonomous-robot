import { Test, TestingModule } from '@nestjs/testing';
import { MissionRuntimeService, MissionCreatePayload, MissionRuntimeSnapshot } from './mission-runtime.service';
import { MissionLogEntry } from '@common/interfaces/mission-log-entry';
import { MissionDatabaseService } from '../mission-database/mission-database.service';
import { SocketService } from '../socket/socket.service';

describe('MissionRuntimeService', () => {
  let service: MissionRuntimeService;

  beforeEach(async () => {
    const module: TestingModule = await Test.createTestingModule({
      providers: [
        MissionRuntimeService,
        { provide: MissionDatabaseService, useValue: { createMission: jest.fn() } },
        { provide: SocketService, useValue: { getMaps: jest.fn().mockReturnValue({ limo1: {}, limo2: {} }) } },
      ],
    }).compile();

    service = module.get<MissionRuntimeService>(MissionRuntimeService);
  });

  it('should be defined', () => {
    expect(service).toBeDefined();
  });

  describe('createMission', () => {
    it('should create a mission successfully', () => {
      const payload: MissionCreatePayload = {
        missionName: 'TestMission',
        robots: ['limo1'],
        mode: 'SIMULATION',
        distance: 5,
        durationSec: 100,
      };

      const mission = service.createMission('socket1', payload);

      expect(mission.missionId).toBeDefined();
      expect(mission.socketId).toBe('socket1');
      expect(mission.missionName).toBe('TestMission');
      expect(mission.mode).toBe('SIMULATION');
      expect(mission.distance).toBe(5);
      expect(service.getActiveMission()).toBe(mission);
    });

    it('should throw error if a mission is already active', () => {
      const payload: MissionCreatePayload = { missionName: 'M1', robots: ['r1'], mode: 'SIMULATION' };
      service.createMission('socket1', payload);

      expect(() => service.createMission('socket2', payload)).toThrow('Une mission est déjà en cours');
    });

    it('should throw error for invalid payload', () => {
      expect(() => service.createMission('socket1', { missionName: '', robots: [], mode: 'SIMULATION' })).toThrow(
        'Mission payload incomplet'
      );
    });
  });

  describe('updateMission', () => {
    it('should update mission fields', () => {
      const payload: MissionCreatePayload = { missionName: 'M1', robots: ['r1'], mode: 'SIMULATION' };
      const mission = service.createMission('socket1', payload);

      const updated = service.updateMission(mission.missionId, { missionName: 'UpdatedMission', durationSec: 200 });
      expect(updated.missionName).toBe('UpdatedMission');
      expect(updated.durationSec).toBe(200);
      expect(updated.updatedAt).toBeInstanceOf(Date);
    });

    it('should append logs and recompute distance', () => {
      const payload: MissionCreatePayload = { missionName: 'M1', robots: ['r1'], mode: 'SIMULATION' };
      const mission = service.createMission('socket1', payload);

      const log: MissionLogEntry = { timestamp: '2025-01-01', robot: 'r1', category: 'Command', action: 'move', details: { totalDistance: 10 } };
      const updated = service.updateMission(mission.missionId, { logs: [log] });

      expect(updated.logs?.length).toBe(1);
      expect(updated.distance).toBe(10);
    });

    it('should throw error if mission not found', () => {
      expect(() => service.updateMission('invalid', {})).toThrow('Mission introuvable');
    });
  });

  describe('appendLog', () => {
    it('should append log and update distance', () => {
      const mission = service.createMission('socket1', { missionName: 'M1', robots: ['r1'], mode: 'SIMULATION' });
      const log: MissionLogEntry = { timestamp: '2025-01-01', robot: 'r1', category: 'Command', action: 'move', details: { totalDistance: 7 } };
      
      const updated = service.appendLog(mission.missionId, log);
      expect(updated.logs?.length).toBe(1);
      expect(updated.distance).toBe(7);
    });

    it('should throw error if mission not found', () => {
      const log: MissionLogEntry = { timestamp: '2025-01-01', robot: 'r1', category: 'Command', action: 'move', details: { totalDistance: 7 } };
      expect(() => service.appendLog('invalid', log)).toThrow('Mission introuvable');
    });
  });

  describe('completeMission', () => {
    it('should complete mission and clear active mission', () => {
      const mission = service.createMission('socket1', { missionName: 'M1', robots: ['r1'], mode: 'SIMULATION' });
      const completed = service.completeMission(mission.missionId);

      expect(completed.status).toBe('COMPLETED');
      expect(service.getActiveMission()).toBeNull();
      expect(service.getCurrentMode()).toBeNull();
    });

    it('should throw error if mission not found', () => {
      expect(() => service.completeMission('invalid')).toThrow('Mission introuvable');
    });
  });

  describe('clearMissionForSocket', () => {
    it('should clear active mission if socket matches', () => {
      const mission = service.createMission('socket1', { missionName: 'M1', robots: ['r1'], mode: 'SIMULATION' });
      service.clearMissionForSocket('socket1');
      expect(service.getActiveMission()).toBeNull();
    });

    it('should not clear mission for different socket', () => {
      const mission = service.createMission('socket1', { missionName: 'M1', robots: ['r1'], mode: 'SIMULATION' });
      service.clearMissionForSocket('socket2');
      expect(service.getActiveMission()).toBe(mission);
    });
  });

  describe('getCurrentMode', () => {
    it('should return current mode or null', () => {
      expect(service.getCurrentMode()).toBeNull();
      const mission = service.createMission('socket1', { missionName: 'M1', robots: ['r1'], mode: 'REAL' });
      expect(service.getCurrentMode()).toBe('REAL');
    });
  });
});
