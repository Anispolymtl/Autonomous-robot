import { Test, TestingModule } from '@nestjs/testing';
import { StateService } from './state.service';
import { SocketService } from '@app/services/socket/socket.service';
import { NavService } from '@app/services/nav/nav.service';
import * as rclnodejs from 'rclnodejs';

jest.mock('rclnodejs');

describe('StateService', () => {
  let service: StateService;
  let socketService: SocketService;

  const mockCreateSubscription = jest.fn();
  const mockSpin = jest.fn();
  const RobotStateMock = {
    WAIT: 0,
    EXPLORATION: 1,
    NAVIGATION: 2,
    RETURN_TO_BASE: 3,
    CUSTOM_MISSION: 4,
  };

  beforeEach(async () => {
    (rclnodejs.require as unknown as jest.Mock).mockImplementation((pkg: string) => {
      if (pkg === 'limo_interfaces') {
        return { msg: { RobotState: RobotStateMock } };
      }
      return {};
    });

    (rclnodejs.Node as unknown as jest.Mock).mockImplementation((name: string, robotId: string) => {
      return {
        createSubscription: mockCreateSubscription,
        spin: mockSpin,
        name,
        robotId,
      };
    });

    const module: TestingModule = await Test.createTestingModule({
      providers: [
        StateService,
        {
          provide: SocketService,
          useValue: {
            sendStateToAllSockets: jest.fn(),
          },
        },
        {
          provide: NavService,
          useValue: {
            isReturnInProgress: mockIsReturnInProgress,
          },
        },
      ],
    }).compile();

    service = module.get<StateService>(StateService);
    socketService = module.get<SocketService>(SocketService);
  });

  afterEach(() => {
    jest.clearAllMocks();
  });

  it('should be defined', () => {
    expect(service).toBeDefined();
  });

  it('should initialize state listeners for limo1 and limo2', () => {
    service.initStateService();

    expect(rclnodejs.Node).toHaveBeenCalledTimes(2);
    expect(rclnodejs.Node).toHaveBeenCalledWith('state_listener_backend_limo1', 'limo1');
    expect(rclnodejs.Node).toHaveBeenCalledWith('state_listener_backend_limo2', 'limo2');

    expect(mockCreateSubscription).toHaveBeenCalledTimes(2);
    expect(mockCreateSubscription).toHaveBeenCalledWith(
      'limo_interfaces/msg/RobotState',
      '/limo1/robot_state',
      expect.any(Function),
    );
    expect(mockCreateSubscription).toHaveBeenCalledWith(
      'limo_interfaces/msg/RobotState',
      '/limo2/robot_state',
      expect.any(Function),
    );
    expect(mockSpin).toHaveBeenCalledTimes(2);
  });

  it('should call sendStateToAllSockets when a message is received', () => {
    const callbackCaptor: Function[] = [];

    mockCreateSubscription.mockImplementation((_msgType, _topic, callback) => {
      callbackCaptor.push(callback);
    });

    service.initStateService();

    const msg1 = { state: RobotStateMock.EXPLORATION };
    callbackCaptor[0](msg1);
    expect(socketService.sendStateToAllSockets).toHaveBeenCalledWith('limo1', 'Exploration');

    const msg2 = { state: RobotStateMock.RETURN_TO_BASE };
    callbackCaptor[1](msg2);
    expect(socketService.sendStateToAllSockets).toHaveBeenCalledWith('limo2', 'Retour a la base');
  });

  it('should ignore waiting state when return to base is in progress', () => {
    mockIsReturnInProgress.mockImplementation((robot: string) => robot === 'limo1');

    const callbackCaptor: Function[] = [];
    mockCreateSubscription.mockImplementation((_msgType, _topic, callback) => {
      callbackCaptor.push(callback);
    });

    service.initStateService();

    const waitingMsg = { data: 'En attente' };
    callbackCaptor[0](waitingMsg);
    expect(socketService.sendStateToAllSockets).not.toHaveBeenCalledWith('limo1', 'En attente');

    const normalMsg = { data: 'Trajet en cours' };
    callbackCaptor[0](normalMsg);
    expect(socketService.sendStateToAllSockets).toHaveBeenCalledWith('limo1', 'Trajet en cours');
  });
});
