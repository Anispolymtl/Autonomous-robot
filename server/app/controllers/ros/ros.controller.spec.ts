import { Test, TestingModule } from '@nestjs/testing';
import { RosController } from './ros.controller';
import { RosService } from '@app/services/ros/ros.service';

describe('RosController', () => {
  let controller: RosController;
  let rosService: Partial<RosService>;

  beforeEach(async () => {
    rosService = {
      identifyRobot: jest.fn().mockImplementation((id: number) => {
        return Promise.resolve({ success: true, robotId: id });
      }),
    };

    const module: TestingModule = await Test.createTestingModule({
      controllers: [RosController],
      providers: [{ provide: RosService, useValue: rosService }],
    }).compile();

    controller = module.get<RosController>(RosController);
  });

  it('should be defined', () => {
    expect(controller).toBeDefined();
  });

  describe('identify', () => {
    it('should return success for valid ID 1', async () => {
      const result = await controller.identify(1);
      expect(rosService.identifyRobot).toHaveBeenCalledWith(1);
      expect(result).toEqual({ success: true, robotId: 1 });
    });

    it('should return success for valid ID 2', async () => {
      const result = await controller.identify(2);
      expect(rosService.identifyRobot).toHaveBeenCalledWith(2);
      expect(result).toEqual({ success: true, robotId: 2 });
    });

    it('should return failure for invalid ID 0', async () => {
      const result = await controller.identify(0);
      expect(result).toEqual({ success: false, message: 'ID de robot invalide' });
      expect(rosService.identifyRobot).not.toHaveBeenCalled();
    });

    it('should return failure for invalid ID 3', async () => {
      const result = await controller.identify(3);
      expect(result).toEqual({ success: false, message: 'ID de robot invalide' });
      expect(rosService.identifyRobot).not.toHaveBeenCalled();
    });

    it('should return failure when ID is undefined', async () => {
      const result = await controller.identify(undefined);
      expect(result).toEqual({ success: false, message: 'ID de robot invalide' });
      expect(rosService.identifyRobot).not.toHaveBeenCalled();
    });
  });
});
