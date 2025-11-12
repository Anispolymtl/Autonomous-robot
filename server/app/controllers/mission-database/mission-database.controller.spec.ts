import { Test, TestingModule } from '@nestjs/testing';
import { MissionDatabaseController } from './mission-database.controller';
import { MissionDatabaseService } from '@app/services/mission-database/mission-database.service';
import { CreateMissionDto } from '@app/model/dto/mission/create-mission.dto';
import { UpdateMissionDto } from '@app/model/dto/mission/update-mission.dto';
import { Mission } from '@app/model/database/mission';
import { Response } from 'express';

describe('MissionDatabaseController', () => {
  let controller: MissionDatabaseController;
  let service: Partial<Record<keyof MissionDatabaseService, jest.Mock>>;

  beforeEach(async () => {
    service = {
      getAllMissions: jest.fn(),
      getMissionById: jest.fn(),
      getMissionsByRobot: jest.fn(),
      getMissionsByMode: jest.fn(),
      getMissionStats: jest.fn(),
      createMission: jest.fn(),
      updateMission: jest.fn(),
      deleteMission: jest.fn(),
    };

    const module: TestingModule = await Test.createTestingModule({
      controllers: [MissionDatabaseController],
      providers: [{ provide: MissionDatabaseService, useValue: service }],
    }).compile();

    controller = module.get<MissionDatabaseController>(MissionDatabaseController);
  });

  const mockResponse = () => {
    const res: Partial<Response> = {};
    res.status = jest.fn().mockReturnValue(res);
    res.json = jest.fn().mockReturnValue(res);
    res.send = jest.fn().mockReturnValue(res);
    return res as Response;
  };

  it('should be defined', () => {
    expect(controller).toBeDefined();
  });

  describe('getAllMissions', () => {
    it('should return all missions', async () => {
      const res = mockResponse();
      const missions: Mission[] = [{ missionName: 'M1', robots: [], mode: 'SIMULATION', distance: 0, durationSec: 0 }];
      service.getAllMissions.mockResolvedValue(missions);

      await controller.getAllMissions(res);
      expect(res.status).toHaveBeenCalledWith(200);
      expect(res.json).toHaveBeenCalledWith(missions);
    });

    it('should handle errors', async () => {
      const res = mockResponse();
      service.getAllMissions.mockRejectedValue(new Error('DB error'));
      await controller.getAllMissions(res);
      expect(res.status).toHaveBeenCalledWith(500);
      expect(res.send).toHaveBeenCalledWith('DB error');
    });
  });

  describe('getMissionById', () => {
    it('should return a mission by ID', async () => {
      const res = mockResponse();
      const mission: Mission = { missionName: 'M1', robots: [], mode: 'SIMULATION', distance: 0, durationSec: 0 };
      service.getMissionById.mockResolvedValue(mission);

      await controller.getMissionById('123', res);
      expect(res.status).toHaveBeenCalledWith(200);
      expect(res.json).toHaveBeenCalledWith(mission);
    });

    it('should return 404 if mission not found', async () => {
      const res = mockResponse();
      service.getMissionById.mockResolvedValue(null);

      await controller.getMissionById('123', res);
      expect(res.status).toHaveBeenCalledWith(404);
      expect(res.send).toHaveBeenCalledWith('Mission not found');
    });

    it('should handle errors', async () => {
      const res = mockResponse();
      service.getMissionById.mockRejectedValue(new Error('DB error'));
      await controller.getMissionById('123', res);
      expect(res.status).toHaveBeenCalledWith(500);
      expect(res.send).toHaveBeenCalledWith('DB error');
    });
  });

  describe('getMissionsByRobot', () => {
    it('should return missions for a robot', async () => {
      const res = mockResponse();
      const missions: Mission[] = [{ missionName: 'M1', robots: ['r1'], mode: 'SIMULATION', distance: 0, durationSec: 0 }];
      service.getMissionsByRobot.mockResolvedValue(missions);

      await controller.getMissionsByRobot('r1', res);
      expect(res.status).toHaveBeenCalledWith(200);
      expect(res.json).toHaveBeenCalledWith(missions);
    });

    it('should handle errors', async () => {
      const res = mockResponse();
      service.getMissionsByRobot.mockRejectedValue(new Error('DB error'));
      await controller.getMissionsByRobot('r1', res);
      expect(res.status).toHaveBeenCalledWith(500);
      expect(res.send).toHaveBeenCalledWith('DB error');
    });
  });

  describe('getMissionsByMode', () => {
    it('should return missions by mode', async () => {
      const res = mockResponse();
      const missions: Mission[] = [{ missionName: 'M1', robots: [], mode: 'REAL', distance: 0, durationSec: 0 }];
      service.getMissionsByMode.mockResolvedValue(missions);

      await controller.getMissionsByMode('REAL', res);
      expect(res.status).toHaveBeenCalledWith(200);
      expect(res.json).toHaveBeenCalledWith(missions);
    });

    it('should handle errors', async () => {
      const res = mockResponse();
      service.getMissionsByMode.mockRejectedValue(new Error('DB error'));
      await controller.getMissionsByMode('REAL', res);
      expect(res.status).toHaveBeenCalledWith(500);
      expect(res.send).toHaveBeenCalledWith('DB error');
    });
  });

  describe('getMissionStats', () => {
    it('should return mission stats', async () => {
      const res = mockResponse();
      const stats = { total: 5 };
      service.getMissionStats.mockResolvedValue(stats);

      await controller.getMissionStats(res);
      expect(res.status).toHaveBeenCalledWith(200);
      expect(res.json).toHaveBeenCalledWith(stats);
    });

    it('should handle errors', async () => {
      const res = mockResponse();
      service.getMissionStats.mockRejectedValue(new Error('DB error'));

      await controller.getMissionStats(res);
      expect(res.status).toHaveBeenCalledWith(500);
      expect(res.send).toHaveBeenCalledWith('DB error');
    });
  });

  describe('createMission', () => {
    it('should handle errors', async () => {
      const res = mockResponse();
      const dto: CreateMissionDto = { missionName: 'M1', robots: [], mode: 'SIMULATION', distance: 0, durationSec: 0 };
      service.createMission.mockRejectedValue(new Error('Invalid'));

      await controller.createMission(dto, res);
      expect(res.status).toHaveBeenCalledWith(400);
      expect(res.send).toHaveBeenCalledWith('Invalid');
    });
  });

  describe('updateMission', () => {
    it('should update mission', async () => {
      const res = mockResponse();
      const dto: UpdateMissionDto = {
        missionName: 'Updated',
        _id: ''
      };
      const mission: Mission = { missionName: 'Updated', robots: [], mode: 'SIMULATION', distance: 0, durationSec: 0 };
      service.updateMission.mockResolvedValue(mission);

      await controller.updateMission(dto, res);
      expect(res.status).toHaveBeenCalledWith(200);
      expect(res.json).toHaveBeenCalledWith(mission);
    });

    it('should return 404 if mission not found', async () => {
      const res = mockResponse();
      const dto: UpdateMissionDto = {
        missionName: 'Updated',
        _id: ''
      };
      service.updateMission.mockResolvedValue(null);

      await controller.updateMission(dto, res);
      expect(res.status).toHaveBeenCalledWith(404);
      expect(res.send).toHaveBeenCalledWith('Mission not found');
    });

    it('should handle errors', async () => {
      const res = mockResponse();
      const dto: UpdateMissionDto = {
        missionName: 'Updated',
        _id: ''
      };
      service.updateMission.mockRejectedValue(new Error('Invalid'));

      await controller.updateMission(dto, res);
      expect(res.status).toHaveBeenCalledWith(400);
      expect(res.send).toHaveBeenCalledWith('Invalid');
    });
  });

  describe('deleteMission', () => {
    it('should delete a mission', async () => {
      const res = mockResponse();
      service.deleteMission.mockResolvedValue(true);

      await controller.deleteMission('123', res);
      expect(res.status).toHaveBeenCalledWith(200);
      expect(res.json).toHaveBeenCalledWith({ message: 'Mission deleted successfully' });
    });

    it('should return 404 if mission not found', async () => {
      const res = mockResponse();
      service.deleteMission.mockResolvedValue(false);

      await controller.deleteMission('123', res);
      expect(res.status).toHaveBeenCalledWith(404);
      expect(res.json).toHaveBeenCalledWith({ message: 'Mission not found' });
    });

    it('should handle errors', async () => {
      const res = mockResponse();
      service.deleteMission.mockRejectedValue(new Error('DB error'));

      await controller.deleteMission('123', res);
      expect(res.status).toHaveBeenCalledWith(500);
      expect(res.json).toHaveBeenCalledWith({ error: 'DB error' });
    });
  });
});
