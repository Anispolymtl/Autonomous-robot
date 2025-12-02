import { Test, TestingModule } from '@nestjs/testing';
import { MissionDatabaseService } from './mission-database.service';
import { getModelToken } from '@nestjs/mongoose';
import { Model } from 'mongoose';
import { Mission } from '@app/model/database/mission';
import { Logger } from '@nestjs/common';
import { CreateMissionDto } from '@app/model/dto/mission/create-mission.dto';
import { UpdateMissionDto } from '@app/model/dto/mission/update-mission.dto';

const createQueryMock = (result: any) => {
  const exec = jest.fn().mockResolvedValue(result);
  return {
    sort: jest.fn().mockReturnThis(),
    skip: jest.fn().mockReturnThis(),
    limit: jest.fn().mockReturnThis(),
    exec,
  };
};

describe('MissionDatabaseService', () => {
  let service: MissionDatabaseService;
  let model: Partial<Record<keyof Model<Mission>, jest.Mock>>;
  let loggerErrorSpy: jest.SpyInstance;

  beforeEach(async () => {
    loggerErrorSpy = jest.spyOn(Logger.prototype, 'error').mockImplementation(() => {});
    model = {
      countDocuments: jest.fn(),
      find: jest.fn(),
      findById: jest.fn(),
      findByIdAndUpdate: jest.fn(),
      findByIdAndDelete: jest.fn(),
      create: jest.fn(),
    };

    const module: TestingModule = await Test.createTestingModule({
      providers: [
        MissionDatabaseService,
        { provide: getModelToken('Mission', 'robot_ops'), useValue: model },
      ],
    }).compile();

    service = module.get<MissionDatabaseService>(MissionDatabaseService);
  });

  afterEach(() => {
    loggerErrorSpy.mockRestore();
  });

  it('should be defined', () => {
    expect(service).toBeDefined();
  });

  describe('getAllMissions', () => {
    it('should return all missions', async () => {
      const missions = [{ missionName: 'M1', robots: [], mode: 'SIMULATION', distance: 0, durationSec: 0 }];
      model.countDocuments.mockResolvedValue(1);
      const query = createQueryMock(missions);
      model.find.mockReturnValue(query);

      const result = await service.getAllMissions(2, 3);
      expect(result).toEqual(missions);
      expect(query.skip).toHaveBeenCalledWith(3);
      expect(query.limit).toHaveBeenCalledWith(2);
    });

    it('should throw error if database fails', async () => {
      model.countDocuments.mockRejectedValue(new Error('DB fail'));
      await expect(service.getAllMissions()).rejects.toThrow('DB fail');
    });

    it('should skip pagination when parameters are not provided', async () => {
      const missions = [{ missionName: 'M1', robots: [], mode: 'SIMULATION', distance: 0, durationSec: 0 }];
      model.countDocuments.mockResolvedValue(1);
      const query = createQueryMock(missions);
      model.find.mockReturnValue(query);

      await service.getAllMissions();

      expect(query.skip).not.toHaveBeenCalled();
      expect(query.limit).not.toHaveBeenCalled();
    });
  });

  describe('getMissionById', () => {
    it('should return mission by ID', async () => {
      const mission = { missionName: 'M1' } as Mission;
      model.findById.mockReturnValue({ exec: () => Promise.resolve(mission) });

      const result = await service.getMissionById('123');
      expect(result).toEqual(mission);
    });
  });

  describe('getMissionsByRobot', () => {
    it('should return missions for a robot', async () => {
      const missions = [{ missionName: 'M1', robots: ['r1'] }] as Mission[];
      const query = createQueryMock(missions);
      model.find.mockReturnValue(query);

      const result = await service.getMissionsByRobot('r1', 1, 0);
      expect(result).toEqual(missions);
      expect(query.skip).toHaveBeenCalledWith(0);
      expect(query.limit).toHaveBeenCalledWith(1);
    });

    it('should allow fetching by robot without pagination', async () => {
      const missions = [{ missionName: 'M1', robots: ['r1'] }] as Mission[];
      const query = createQueryMock(missions);
      model.find.mockReturnValue(query);

      await service.getMissionsByRobot('r1');

      expect(query.skip).not.toHaveBeenCalled();
      expect(query.limit).not.toHaveBeenCalled();
    });
  });

  describe('getMissionsByMode', () => {
    it('should return missions by mode', async () => {
      const missions = [{ missionName: 'M1', mode: 'REAL' }] as Mission[];
      const query = createQueryMock(missions);
      model.find.mockReturnValue(query);

      const result = await service.getMissionsByMode('REAL', 5, 2);
      expect(result).toEqual(missions);
      expect(query.skip).toHaveBeenCalledWith(2);
      expect(query.limit).toHaveBeenCalledWith(5);
    });

    it('should allow fetching by mode without pagination', async () => {
      const missions = [{ missionName: 'M1', mode: 'REAL' }] as Mission[];
      const query = createQueryMock(missions);
      model.find.mockReturnValue(query);

      await service.getMissionsByMode('REAL');

      expect(query.skip).not.toHaveBeenCalled();
      expect(query.limit).not.toHaveBeenCalled();
    });
  });

  describe('createMission', () => {
    it('should create a mission', async () => {
      const dto: CreateMissionDto = {
        missionName: 'M1',
        robots: [],
        mode: 'SIMULATION',
        distance: 0,
        durationSec: 0,
        maps: {
          limo1: { header: {}, info: {}, data: [] },
          limo2: { header: {}, info: {}, data: [] },
        },
      };
      model.create.mockResolvedValue(dto);

      const result = await service.createMission(dto);
      expect(result).toEqual(dto);
    });

    it('should throw error on create failure', async () => {
      model.create.mockRejectedValue(new Error('Create fail'));
      await expect(service.createMission({ missionName: 'M1', robots: [], mode: 'SIMULATION' } as any)).rejects.toThrow('Failed to create mission: Error: Create fail');
    });
  });

  describe('updateMission', () => {
    it('should update mission', async () => {
      const dto: UpdateMissionDto = { _id: '123', missionName: 'Updated' };
      const updated: Mission = { missionName: 'Updated', robots: [], mode: 'SIMULATION', distance: 0, durationSec: 0 };
      model.findByIdAndUpdate.mockReturnValue({ exec: () => Promise.resolve(updated) });

      const result = await service.updateMission(dto);
      expect(result).toEqual(updated);
    });

    it('should throw error on update failure', async () => {
      const dto: UpdateMissionDto = { _id: '123', missionName: 'Updated' };
      model.findByIdAndUpdate.mockReturnValue({ exec: () => Promise.reject(new Error('Update fail')) });

      await expect(service.updateMission(dto)).rejects.toThrow('Failed to update mission: Error: Update fail');
    });
  });

  describe('deleteMission', () => {
    it('should return true if deleted', async () => {
      model.findByIdAndDelete.mockReturnValue({ exec: () => Promise.resolve({}) });
      const result = await service.deleteMission('123');
      expect(result).toBe(true);
    });

    it('should return false if mission not found', async () => {
      model.findByIdAndDelete.mockReturnValue({ exec: () => Promise.resolve(null) });
      const result = await service.deleteMission('123');
      expect(result).toBe(false);
    });

    it('should throw error on delete failure', async () => {
      model.findByIdAndDelete.mockReturnValue({ exec: () => Promise.reject(new Error('Delete fail')) });
      await expect(service.deleteMission('123')).rejects.toThrow('Failed to delete mission: Error: Delete fail');
    });
  });

  describe('getMissionStats', () => {
    it('should return mission statistics', async () => {
      const missions: Mission[] = [
        { missionName: 'M1', robots: ['r1'], mode: 'SIMULATION', distance: 10, durationSec: 5 },
        { missionName: 'M2', robots: ['r2'], mode: 'REAL', distance: 20, durationSec: 15 },
      ];
      model.find.mockReturnValue({ exec: () => Promise.resolve(missions) });

      const stats = await service.getMissionStats();
      expect(stats).toEqual({
        total: 2,
        byRobot: { r1: 1, r2: 1 },
        byMode: { SIMULATION: 1, REAL: 1 },
        totalDistance: 30,
        averageDuration: 10,
      });
    });
  });
});
