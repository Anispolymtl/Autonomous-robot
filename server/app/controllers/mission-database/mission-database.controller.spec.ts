import { Test, TestingModule } from '@nestjs/testing';
import { MissionDatabaseController } from './mission-database.controller';
import { MissionDatabaseService } from '@app/services/mission-database/mission-database.service';
import { Response } from 'express';
import { CreateMissionDto } from '@app/model/dto/mission/create-mission.dto';
import { UpdateMissionDto } from '@app/model/dto/mission/update-mission.dto';

describe('MissionDatabaseController', () => {
  let controller: MissionDatabaseController;
  let service: Partial<MissionDatabaseService>;
  let mockResponse: Partial<Response>;

  beforeEach(async () => {
    service = {
      getAllMissions: jest.fn().mockResolvedValue([{ id: '1', name: 'Test Mission' }]),
      getMissionById: jest.fn().mockResolvedValue({ id: '1', name: 'Test Mission' }),
      getMissionsByRobot: jest.fn().mockResolvedValue([{ id: '1', name: 'Test Mission' }]),
      getMissionsByMode: jest.fn().mockResolvedValue([{ id: '1', name: 'Test Mission' }]),
      getMissionStats: jest.fn().mockResolvedValue({ total: 1 }),
      createMission: jest.fn().mockResolvedValue({ id: '1', name: 'Created Mission' }),
      updateMission: jest.fn().mockResolvedValue({ id: '1', name: 'Updated Mission' }),
      deleteMission: jest.fn().mockResolvedValue(true),
      populateDatabase: jest.fn().mockResolvedValue({ populated: true }),
    };

    mockResponse = {
      status: jest.fn().mockReturnThis(),
      json: jest.fn().mockReturnThis(),
      send: jest.fn().mockReturnThis(),
    };

    const module: TestingModule = await Test.createTestingModule({
      controllers: [MissionDatabaseController],
      providers: [{ provide: MissionDatabaseService, useValue: service }],
    }).compile();

    controller = module.get<MissionDatabaseController>(MissionDatabaseController);
  });

  it('should be defined', () => {
    expect(controller).toBeDefined();
  });

  describe('getAllMissions', () => {
    it('should return all missions', async () => {
      await controller.getAllMissions(mockResponse as Response);
      expect(service.getAllMissions).toHaveBeenCalled();
      expect(mockResponse.status).toHaveBeenCalledWith(200);
      expect(mockResponse.json).toHaveBeenCalledWith([{ id: '1', name: 'Test Mission' }]);
    });
  });

  describe('getMissionById', () => {
    it('should return mission when found', async () => {
      await controller.getMissionById('1', mockResponse as Response);
      expect(service.getMissionById).toHaveBeenCalledWith('1');
      expect(mockResponse.status).toHaveBeenCalledWith(200);
      expect(mockResponse.json).toHaveBeenCalledWith({ id: '1', name: 'Test Mission' });
    });

    it('should return 404 when mission not found', async () => {
      (service.getMissionById as jest.Mock).mockResolvedValueOnce(null);
      await controller.getMissionById('2', mockResponse as Response);
      expect(mockResponse.status).toHaveBeenCalledWith(404);
      expect(mockResponse.send).toHaveBeenCalledWith('Mission not found');
    });
  });

  describe('getMissionsByRobot', () => {
    it('should return missions filtered by robot', async () => {
      await controller.getMissionsByRobot('Robo1', mockResponse as Response);
      expect(service.getMissionsByRobot).toHaveBeenCalledWith('Robo1');
      expect(mockResponse.status).toHaveBeenCalledWith(200);
    });
  });

  describe('getMissionsByMode', () => {
    it('should return missions filtered by mode', async () => {
      await controller.getMissionsByMode('AUTO', mockResponse as Response);
      expect(service.getMissionsByMode).toHaveBeenCalledWith('AUTO');
      expect(mockResponse.status).toHaveBeenCalledWith(200);
    });
  });

  describe('getMissionStats', () => {
    it('should return mission stats', async () => {
      await controller.getMissionStats(mockResponse as Response);
      expect(service.getMissionStats).toHaveBeenCalled();
      expect(mockResponse.status).toHaveBeenCalledWith(200);
      expect(mockResponse.json).toHaveBeenCalledWith({ total: 1 });
    });
  });

  describe('createMission', () => {
    it('should create a mission', async () => {
      const dto: CreateMissionDto = {
        missionName: 'New Mission',
        durationSec: 60,
        robots: ['Robo1'],
        mode: 'AUTO',
        distance: 100,
      };
      await controller.createMission(dto, mockResponse as Response);
      expect(service.createMission).toHaveBeenCalledWith(dto);
      expect(mockResponse.status).toHaveBeenCalledWith(201);
      expect(mockResponse.json).toHaveBeenCalledWith({ id: '1', name: 'Created Mission' });
    });
  });

  describe('updateMission', () => {
    it('should update a mission', async () => {
      const dto: UpdateMissionDto = {
        _id: '1',
        missionName: 'Updated Mission',
        durationSec: 60,
        robots: ['Robo1'],
        mode: 'AUTO',
        distance: 150,
      };
      await controller.updateMission(dto, mockResponse as Response);
      expect(service.updateMission).toHaveBeenCalledWith(dto);
      expect(mockResponse.status).toHaveBeenCalledWith(200);
      expect(mockResponse.json).toHaveBeenCalledWith({ id: '1', name: 'Updated Mission' });
    });

    it('should return 404 if mission does not exist', async () => {
      (service.updateMission as jest.Mock).mockResolvedValueOnce(null);
      const dto: UpdateMissionDto = {
        _id: '2',
        missionName: 'Missing Mission',
        durationSec: 60,
        robots: ['Robo2'],
        mode: 'AUTO',
        distance: 100,
      };
      await controller.updateMission(dto, mockResponse as Response);
      expect(mockResponse.status).toHaveBeenCalledWith(404);
      expect(mockResponse.send).toHaveBeenCalledWith('Mission not found');
    });
  });

  describe('deleteMission', () => {
    it('should delete mission', async () => {
      await controller.deleteMission('1', mockResponse as Response);
      expect(service.deleteMission).toHaveBeenCalledWith('1');
      expect(mockResponse.status).toHaveBeenCalledWith(200);
      expect(mockResponse.json).toHaveBeenCalledWith({ message: 'Mission deleted successfully' });
    });

    it('should return 404 if mission not found', async () => {
      (service.deleteMission as jest.Mock).mockResolvedValueOnce(false);
      await controller.deleteMission('2', mockResponse as Response);
      expect(mockResponse.status).toHaveBeenCalledWith(404);
      expect(mockResponse.json).toHaveBeenCalledWith({ message: 'Mission not found' });
    });
  });

  describe('populateDatabase', () => {
    it('should populate database with force=false', async () => {
      await controller.populateDatabase({}, mockResponse as Response);
      expect(service.populateDatabase).toHaveBeenCalledWith(false);
      expect(mockResponse.status).toHaveBeenCalledWith(201);
      expect(mockResponse.json).toHaveBeenCalledWith({ populated: true });
    });

    it('should populate database with force=true', async () => {
      await controller.populateDatabase({ force: true }, mockResponse as Response);
      expect(service.populateDatabase).toHaveBeenCalledWith(true);
      expect(mockResponse.status).toHaveBeenCalledWith(201);
    });
  });
});
