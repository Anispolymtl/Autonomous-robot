import { Test, TestingModule } from '@nestjs/testing';
import { StateService } from './state.service';
import { SocketService } from '@app/services/socket/socket.service';
import * as rclnodejs from 'rclnodejs';

jest.mock('rclnodejs');

describe('StateService', () => {
  let service: StateService;
  let socketService: SocketService;

  const mockCreateSubscription = jest.fn();
  const mockSpin = jest.fn();

  beforeEach(async () => {
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
    expect(rclnodejs.Node).toHaveBeenCalledWith('state_listener_backend', 'limo1');
    expect(rclnodejs.Node).toHaveBeenCalledWith('state_listener_backend', 'limo2');

    expect(mockCreateSubscription).toHaveBeenCalledTimes(2);
    expect(mockSpin).toHaveBeenCalledTimes(2);
  });

  it('should call sendStateToAllSockets when a message is received', () => {
    const callbackCaptor: Function[] = [];

    mockCreateSubscription.mockImplementation((_msgType, _topic, callback) => {
      callbackCaptor.push(callback);
    });

    service.initStateService();

    const msg1 = { data: 'RUNNING' };
    callbackCaptor[0](msg1);
    expect(socketService.sendStateToAllSockets).toHaveBeenCalledWith('limo1', 'RUNNING');

    const msg2 = { data: 'COMPLETED' };
    callbackCaptor[1](msg2);
    expect(socketService.sendStateToAllSockets).toHaveBeenCalledWith('limo2', 'COMPLETED');
  });
});
