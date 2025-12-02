import { Test, TestingModule } from '@nestjs/testing';
import { MappingSerivce } from './mapping.service';
import { SocketService } from '../socket/socket.service';
import * as rclnodejs from 'rclnodejs';

jest.mock('rclnodejs', () => ({
  Node: jest.fn().mockImplementation(() => ({
    createSubscription: jest.fn(),
    spin: jest.fn(),
  })),
}));

describe('MappingService', () => {
  let service: MappingSerivce;
  let socketService: Partial<SocketService>;

  beforeEach(async () => {
    socketService = {
      sendMapToAllSockets: jest.fn(),
      sendPoseToAllSockets: jest.fn(),
    };

    const module: TestingModule = await Test.createTestingModule({
      providers: [
        MappingSerivce,
        { provide: SocketService, useValue: socketService },
      ],
    }).compile();

    service = module.get<MappingSerivce>(MappingSerivce);
  });

  it('should be defined', () => {
    expect(service).toBeDefined();
  });

  describe('initialiseMappingService', () => {
    it('should setup mapping and pose listeners for both robots', () => {
      const setupMappingSpy = jest.spyOn<any, any>(service, 'setupMappingListner');
      const setupPoseSpy = jest.spyOn<any, any>(service, 'setupPoseListners');

      service.initialiseMappingService();

      expect(setupMappingSpy).toHaveBeenCalledWith('limo1');
      expect(setupMappingSpy).toHaveBeenCalledWith('limo2');
      expect(setupPoseSpy).toHaveBeenCalledWith('limo1');
      expect(setupPoseSpy).toHaveBeenCalledWith('limo2');
    });
  });

  describe('setupMappingListner', () => {
    it('should create rclnodejs Node, create subscription and spin', () => {
      const spyNode = jest.spyOn(rclnodejs, 'Node');
      const robotId = 'limo1';

      service['setupMappingListner'](robotId);

      expect(spyNode).toHaveBeenCalledWith('mapping_listener_backend', robotId);
      const nodeInstance = service['mappingNode'] as any;
      expect(nodeInstance.createSubscription).toHaveBeenCalledWith(
        'nav_msgs/msg/OccupancyGrid',
        'map',
        expect.any(Function)
      );
      expect(nodeInstance.spin).toHaveBeenCalled();
    });
  });

  describe('setupPoseListners', () => {
    it('should create rclnodejs Node, create subscription and spin', () => {
      const spyNode = jest.spyOn(rclnodejs, 'Node');
      const robotId = 'limo2';

      service['setupPoseListners'](robotId);

      expect(spyNode).toHaveBeenCalledWith('pose_listener_backend', robotId);
      const nodeInstance = service['poseNode'] as any;
      expect(nodeInstance.createSubscription).toHaveBeenCalledWith(
        'geometry_msgs/msg/PoseStamped',
        'current_pose',
        expect.any(Function)
      );
      expect(nodeInstance.spin).toHaveBeenCalled();
    });
  });
});
