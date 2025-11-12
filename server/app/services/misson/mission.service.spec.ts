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
});
