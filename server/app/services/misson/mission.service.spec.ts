import { Test, TestingModule } from '@nestjs/testing';
import { MissionService } from './mission.service';
import * as rclnodejs from 'rclnodejs';

jest.mock('rclnodejs', () => {
  const sendGoalMock = jest.fn().mockResolvedValue({ accepted: true, getResult: jest.fn() });
  const waitForServerMock = jest.fn().mockResolvedValue(true);
  const spinMock = jest.fn();

  const ActionClientMock = jest.fn().mockImplementation(() => ({
    waitForServer: waitForServerMock,
    sendGoal: sendGoalMock,
  }));

  const NodeMock = jest.fn().mockImplementation(() => ({
    spin: spinMock,
  }));

  return {
    Node: NodeMock,
    ActionClient: ActionClientMock,
    require: jest.fn().mockReturnValue({
      action: { DoMission: { Goal: jest.fn().mockImplementation(() => ({})) } },
    }),
  };
});

describe('MissionService', () => {
  let service: MissionService;

  beforeEach(async () => {
    const module: TestingModule = await Test.createTestingModule({
      providers: [MissionService],
    }).compile();

    service = module.get<MissionService>(MissionService);
  });

  it('should be defined', () => {
    expect(service).toBeDefined();
  });

  it('should initialize ROS nodes and clients on module init', async () => {
    await service.onModuleInit();
    expect((rclnodejs.Node as unknown as jest.Mock).mock.calls.length).toBe(2);
    expect((rclnodejs.ActionClient as jest.Mock).mock.calls.length).toBe(2);
    expect(service['missionNode1']).toBeDefined();
    expect(service['missionNode2']).toBeDefined();
    expect(service['missionClient1']).toBeDefined();
    expect(service['missionClient2']).toBeDefined();
  });

  it('should start mission if clients are initialized', async () => {
    await service.onModuleInit();
    await service.startMission();

    expect(service['goalHandle1']).toBeDefined();
    expect(service['goalHandle2']).toBeDefined();
  });

  it('should not start mission if clients are not initialized', async () => {
    const consoleSpy = jest.spyOn(console, 'error').mockImplementation();
    await service.startMission();
    expect(consoleSpy).toHaveBeenCalledWith('Client ROS2 non initialisé');
    consoleSpy.mockRestore();
  });

  it('should stop mission if a mission is active', async () => {
    await service.onModuleInit();
    await service.startMission();

    const cancelMock1 = jest.fn().mockResolvedValue(undefined);
    const cancelMock2 = jest.fn().mockResolvedValue(undefined);
    service['goalHandle1'].cancelGoal = cancelMock1;
    service['goalHandle2'].cancelGoal = cancelMock2;

    const consoleSpy = jest.spyOn(console, 'log').mockImplementation();
    await service.stopMission();
    expect(service['goalHandle1']).toBeNull();
    expect(service['goalHandle2']).toBeNull();
    consoleSpy.mockRestore();
  });

  it('should warn when stopping mission if no mission active', async () => {
    await service.onModuleInit();
    const consoleSpy = jest.spyOn(console, 'warn').mockImplementation();
    await service.stopMission();
    expect(consoleSpy).toHaveBeenCalledWith('Aucune mission en cours à annuler');
    consoleSpy.mockRestore();
  });
});
