import { Test, TestingModule } from '@nestjs/testing';
import { MissionDatabaseService } from './mission-database.service';
import { getModelToken } from '@nestjs/mongoose';
import { Mission } from '@app/model/database/mission';

describe('MissionDatabaseService', () => {
  let service: MissionDatabaseService;
  let mockModel: any;

  beforeEach(async () => {
    mockModel = {
      find: jest.fn().mockReturnThis(),
      sort: jest.fn().mockReturnThis(),
      exec: jest.fn(),
      create: jest.fn(),
      findById: jest.fn(),
      findByIdAndUpdate: jest.fn(),
      findByIdAndDelete: jest.fn(),
      countDocuments: jest.fn(),
      deleteMany: jest.fn(),
      insertMany: jest.fn(),
    };

    const module: TestingModule = await Test.createTestingModule({
      providers: [
        MissionDatabaseService,
        { provide: getModelToken(Mission.name, 'robot_ops'), useValue: mockModel },
      ],
    }).compile();

    service = module.get<MissionDatabaseService>(MissionDatabaseService);
  });

  it('should be defined', () => {
    expect(service).toBeDefined();
  });

  describe('getMissionById', () => {
    it('should return a mission', async () => {
      const mission = { missionName: 'A' };
      mockModel.findById.mockReturnValueOnce({ exec: jest.fn().mockResolvedValueOnce(mission) });

      const result = await service.getMissionById('id1');
      expect(result).toEqual(mission);
    });
  });

  describe('getMissionsByRobot', () => {
    it('should return missions for a robot', async () => {
      const missions = [{ missionName: 'B' }];
      mockModel.find.mockReturnValueOnce({ sort: jest.fn().mockReturnThis(), exec: jest.fn().mockResolvedValueOnce(missions) });

      const result = await service.getMissionsByRobot('Atlas-R2');
      expect(result).toEqual(missions);
    });
  });

  describe('createMission', () => {
    it('should create a mission', async () => {
      const dto = { missionName: 'C', robots: [], mode: 'REAL', distance: 0, durationSec: 0 };
      mockModel.create.mockResolvedValueOnce(dto);

      const result = await service.createMission(dto);
      expect(result).toEqual(dto);
      expect(mockModel.create).toHaveBeenCalledWith(dto);
    });
  });

  describe('updateMission', () => {
    it('should update a mission', async () => {
      const dto = { _id: 'id1', missionName: 'Updated', robots: [], mode: 'REAL', distance: 0, durationSec: 0 };
      const updated = { ...dto };
      mockModel.findByIdAndUpdate.mockReturnValueOnce({ exec: jest.fn().mockResolvedValueOnce(updated) });

      const result = await service.updateMission(dto);
      expect(result).toEqual(updated);
      expect(mockModel.findByIdAndUpdate).toHaveBeenCalledWith(dto._id, expect.any(Object), { new: true });
    });
  });

  describe('deleteMission', () => {
    it('should delete a mission', async () => {
      mockModel.findByIdAndDelete.mockReturnValueOnce({ exec: jest.fn().mockResolvedValueOnce({}) });

      const result = await service.deleteMission('id1');
      expect(result).toBe(true);
    });

    it('should return false if mission not found', async () => {
      mockModel.findByIdAndDelete.mockReturnValueOnce({ exec: jest.fn().mockResolvedValueOnce(null) });
      const result = await service.deleteMission('id2');
      expect(result).toBe(false);
    });
  });

  describe('getMissionStats', () => {
    it('should compute stats', async () => {
      const missions = [
        { robots: ['A'], mode: 'SIM', distance: 100, durationSec: 10 },
        { robots: ['A', 'B'], mode: 'REAL', distance: 200, durationSec: 20 },
      ];
      mockModel.find.mockReturnValueOnce({ exec: jest.fn().mockResolvedValueOnce(missions) });

      const stats = await service.getMissionStats();
      expect(stats.total).toBe(2);
      expect(stats.byRobot.A).toBe(2);
      expect(stats.byRobot.B).toBe(1);
      expect(stats.byMode.SIM).toBe(1);
      expect(stats.byMode.REAL).toBe(1);
      expect(stats.totalDistance).toBe(300);
      expect(stats.averageDuration).toBe(15);
    });
  });

  describe('populateDatabase', () => {
    it('should populate database if empty', async () => {
      mockModel.countDocuments.mockResolvedValueOnce(0);
      mockModel.insertMany.mockResolvedValueOnce([{ _id: '1' }, { _id: '2' }]);

      const result = await service.populateDatabase();
      expect(result.created).toBe(2);
    });

    it('should skip if not empty and force=false', async () => {
      mockModel.countDocuments.mockResolvedValueOnce(1);

      const result = await service.populateDatabase(false);
      expect(result.created).toBe(0);
    });

    it('should force populate if force=true', async () => {
      mockModel.countDocuments.mockResolvedValueOnce(1);
      mockModel.deleteMany.mockResolvedValueOnce({});
      mockModel.insertMany.mockResolvedValueOnce([{ _id: '1' }]);

      const result = await service.populateDatabase(true);
      expect(result.created).toBe(1);
    });
  });
});
