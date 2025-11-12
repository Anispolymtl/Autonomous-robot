import { Test, TestingModule } from '@nestjs/testing';
import { MissionController } from './mission.controller';
import { MissionService } from '@app/services/misson/mission.service';
import { MissionRuntimeService } from '@app/services/mission-runtime/mission-runtime.service';

describe('MissionController', () => {
  let controller: MissionController;
  let missionService: Partial<MissionService>;
  let missionRuntimeService: Partial<MissionRuntimeService>;

  beforeEach(async () => {
    missionService = {
      startMission: jest.fn().mockResolvedValue('Mission started'),
      stopMission: jest.fn().mockResolvedValue('Mission stopped'),
    };

    missionRuntimeService = {
      getCurrentMode: jest.fn().mockReturnValue('AUTO'),
      getActiveMission: jest.fn().mockReturnValue({ id: 1, name: 'Test Mission' }),
    };

    const module: TestingModule = await Test.createTestingModule({
      controllers: [MissionController],
      providers: [
        { provide: MissionService, useValue: missionService },
        { provide: MissionRuntimeService, useValue: missionRuntimeService },
      ],
    }).compile();

    controller = module.get<MissionController>(MissionController);
  });

  it('should be defined', () => {
    expect(controller).toBeDefined();
  });

  describe('start', () => {
    it('should start the mission', async () => {
      const result = await controller.start();
      expect(result).toBe('Mission started');
      expect(missionService.startMission).toHaveBeenCalled();
    });
  });

  describe('stop', () => {
    it('should stop the mission', async () => {
      const result = await controller.stop();
      expect(result).toBe('Mission stopped');
      expect(missionService.stopMission).toHaveBeenCalled();
    });
  });

  describe('getCurrentMode', () => {
    it('should return current mode', () => {
      const result = controller.getCurrentMode();
      expect(result).toEqual({ mode: 'AUTO' });
      expect(missionRuntimeService.getCurrentMode).toHaveBeenCalled();
    });
  });

  describe('getActiveMission', () => {
    it('should return active mission', () => {
      const result = controller.getActiveMission();
      expect(result).toEqual({ id: 1, name: 'Test Mission' });
      expect(missionRuntimeService.getActiveMission).toHaveBeenCalled();
    });
  });
});
