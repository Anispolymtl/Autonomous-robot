import { Test, TestingModule } from '@nestjs/testing';
import { NavService } from '@app/services/nav/nav.service';
import { SocketService } from '@app/services/socket/socket.service';
import * as rclnodejs from 'rclnodejs';

jest.mock('rclnodejs', () => {
  const mockActionClient = jest.fn().mockImplementation(() => ({
    waitForServer: jest.fn().mockResolvedValue(true),
    sendGoal: jest.fn().mockResolvedValue({
      getResult: jest.fn().mockResolvedValue({ status: 'SUCCEEDED' }),
    }),
  }));

  const SetRobotStateMock = function () {};
  // @ts-ignore
  SetRobotStateMock.Request = function () {
    this.state = 0;
  };

  return {
    ActionClient: mockActionClient,
    require: jest.fn().mockImplementation((pkg: string) => {
      if (pkg === 'limo_interfaces') {
        return {
          msg: {
            RobotState: {
              WAIT: 0,
              EXPLORATION: 1,
              NAVIGATION: 2,
              RETURN_TO_BASE: 3,
              CUSTOM_MISSION: 4,
            },
          },
          srv: {
            SetRobotState: SetRobotStateMock,
          },
        };
      }
      return {};
    }),
  };
});

describe('NavService', () => {
    let service: NavService;
    let socketService: SocketService;

  beforeEach(async () => {
    const module: TestingModule = await Test.createTestingModule({
      providers: [
        NavService,
        {
                    provide: SocketService,
                    useValue: { sendPointsToAllSockets: jest.fn() },
                },
            ],
        }).compile();

    service = module.get<NavService>(NavService);
    socketService = module.get<SocketService>(SocketService);
  });

  const createFakeNode = () =>
    ({
      createClient: jest.fn(() => ({
        sendRequest: jest.fn((_req, cb) => cb({ success: true, message: 'ok' })),
        waitForService: jest.fn().mockResolvedValue(true),
      })),
    } as unknown as rclnodejs.Node);

    it('should be defined', () => {
        expect(service).toBeDefined();
    });

    describe('initNavService', () => {
    it('should initialize nav clients', () => {
            const node1 = createFakeNode();
            const node2 = createFakeNode();
            service.initNavService(node1, node2);
            expect((rclnodejs.ActionClient as jest.Mock).mock.calls.length).toBe(2);
    });
  });

  describe('addPoint', () => {
    beforeEach(() => {
      const fakeNode1 = createFakeNode();
      const fakeNode2 = createFakeNode();
      service.initNavService(fakeNode1, fakeNode2);
    });

    it('should add a point to limo1', () => {
      const payload: { robot: 'limo1'; point: { x: number; y: number } } = {
        robot: 'limo1',
        point: { x: 1, y: 2 },
      };
      service.addPoint(payload);
      expect(socketService.sendPointsToAllSockets).toHaveBeenCalledWith(
        'limo1',
        [{ x: 1, y: 2 }],
      );
    });

    it('should add a point to limo2', () => {
      const payload: { robot: 'limo2'; point: { x: number; y: number } } = {
        robot: 'limo2',
        point: { x: 3, y: 4 },
      };
      service.addPoint(payload);
      expect(socketService.sendPointsToAllSockets).toHaveBeenCalledWith(
        'limo2',
        [{ x: 3, y: 4 }],
      );
    });
  });

  describe('removePoint', () => {
    beforeEach(() => {
      const fakeNode1 = createFakeNode();
      const fakeNode2 = createFakeNode();
      service.initNavService(fakeNode1, fakeNode2);

      service.addPoint({ robot: 'limo1', point: { x: 1, y: 2 } });
      service.addPoint({ robot: 'limo1', point: { x: 3, y: 4 } });
    });

    it('should remove a point by index', () => {
      const payload: { robot: 'limo1'; index: number } = {
        robot: 'limo1',
        index: 0,
      };
      service.removePoint(payload);
      expect(socketService.sendPointsToAllSockets).toHaveBeenCalledWith(
        'limo1',
        [{ x: 3, y: 4 }],
      );
    });

    it('should not remove point with invalid index', () => {
      const payload: { robot: 'limo1'; index: number } = {
        robot: 'limo1',
        index: 5,
      };
      const result = service.removePoint(payload);
      expect(result).toEqual([]);
    });
  });
});
