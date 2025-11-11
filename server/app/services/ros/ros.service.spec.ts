import { Test, TestingModule } from '@nestjs/testing';
import { RosService } from './ros.service';
import { NavService } from '../nav/nav.service';
import { StateService } from '../state/state.service';
import { MappingSerivce } from '../mapping/mapping.service';
import * as rclnodejs from 'rclnodejs';

jest.mock('rclnodejs');

describe('RosService', () => {
  let service: RosService;
  let navService: NavService;
  let stateService: StateService;
  let mappingService: MappingSerivce;

  const mockSpin = jest.fn();
  const mockCreateClient = jest.fn();
  const mockSendRequest = jest.fn();

  beforeEach(async () => {
    (rclnodejs.require as jest.Mock).mockImplementation((pkg: string) => {
      if (pkg === 'std_srvs') {
        return { srv: { Trigger: { Request: jest.fn() } } };
      }
      return {};
    });

    (rclnodejs.Node as unknown as jest.Mock).mockImplementation((name: string, robotId: string) => {
      return {
        createClient: mockCreateClient,
        spin: mockSpin,
        name,
        robotId,
      };
    });

    mockCreateClient.mockImplementation(() => ({
      sendRequest: mockSendRequest,
    }));

    const module: TestingModule = await Test.createTestingModule({
      providers: [
        RosService,
        { provide: NavService, useValue: { initNavService: jest.fn() } },
        { provide: StateService, useValue: { initStateService: jest.fn() } },
        { provide: MappingSerivce, useValue: { initialiseMappingService: jest.fn() } },
      ],
    }).compile();

    service = module.get<RosService>(RosService);
    navService = module.get<NavService>(NavService);
    stateService = module.get<StateService>(StateService);
    mappingService = module.get<MappingSerivce>(MappingSerivce);
  });

  afterEach(() => {
    jest.clearAllMocks();
  });

  it('should be defined', () => {
    expect(service).toBeDefined();
  });

  it('should initialize ROS nodes and services on onModuleInit', async () => {
    await service.onModuleInit();

    expect(rclnodejs.init).toHaveBeenCalled();
    expect(rclnodejs.Node).toHaveBeenCalledTimes(2);
    expect(navService.initNavService).toHaveBeenCalledTimes(1);
    expect(stateService.initStateService).toHaveBeenCalledTimes(1);
    expect(mappingService.initialiseMappingService).toHaveBeenCalledTimes(1);
    expect(mockSpin).toHaveBeenCalledTimes(2);
  });

  it('should call identifyClient.sendRequest and resolve response', async () => {
    await service.onModuleInit();

    const fakeResponse = { success: true, message: 'Robot identifié' };
    mockSendRequest.mockImplementation((_req: any, cb: Function) => cb(fakeResponse));

    const result = await service.identifyRobot(1);

    expect(result).toEqual({ success: true, message: 'Robot identifié' });
    expect(mockSendRequest).toHaveBeenCalledTimes(1);
  });
});
