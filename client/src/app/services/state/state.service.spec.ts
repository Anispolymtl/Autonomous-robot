import { TestBed } from '@angular/core/testing';
import { MissionStateService } from './state.service';
import { SocketService } from '@app/services/socket/socket.service';

describe('MissionStateService', () => {
  let service: MissionStateService;
  let socketServiceMock: any;

  beforeEach(() => {
    socketServiceMock = {
      isSocketAlive: jasmine.createSpy('isSocketAlive'),
      connect: jasmine.createSpy('connect'),
      get getSocket() {
        return this.socket as any;
      },
      socket: {
        once: jasmine.createSpy('once'),
        on: jasmine.createSpy('on'),
      },
      on: jasmine.createSpy('on'),
      disconnect: jasmine.createSpy('disconnect'),
    };

    TestBed.configureTestingModule({
      providers: [
        MissionStateService,
        { provide: SocketService, useValue: socketServiceMock },
      ],
    });

    service = TestBed.inject(MissionStateService);
  });

  it('should be created', () => {
    expect(service).toBeTruthy();
  });

  describe('connectToSocket', () => {
    it('should connect and configure socket when socket is not alive', () => {
      socketServiceMock.isSocketAlive.and.returnValue(false);
      service.connectToSocket();
      expect(socketServiceMock.connect).toHaveBeenCalledWith('client');
      expect(socketServiceMock.socket.once).toHaveBeenCalledWith('connect', jasmine.any(Function));
    });

    it('should configure socket when socket is already alive', () => {
      socketServiceMock.isSocketAlive.and.returnValue(true);
      service.connectToSocket();
      expect(socketServiceMock.connect).not.toHaveBeenCalled();
    });
  });

  describe('configureStateSocketFeatures', () => {
    it('should update limo1 state on stateUpdate', (done) => {
      const callbackMap: Record<string, Function> = {};
      socketServiceMock.on.and.callFake((event: string, cb: Function) => {
        callbackMap[event] = cb;
      });

      (service as any).configureStateSocketFeatures();

      callbackMap['stateUpdate']({ robot: 'limo1', state: 'Moving' });

      service.getLimo1State$().subscribe((state) => {
        expect(state).toBe('Moving');
        done();
      });
    });

    it('should update limo2 state on stateUpdate', (done) => {
      const callbackMap: Record<string, Function> = {};
      socketServiceMock.on.and.callFake((event: string, cb: Function) => {
        callbackMap[event] = cb;
      });

      (service as any).configureStateSocketFeatures();

      callbackMap['stateUpdate']({ robot: 'limo2', state: 'Idle' });

      service.getLimo2State$().subscribe((state) => {
        expect(state).toBe('Idle');
        done();
      });
    });

    it('should ignore invalid payload', (done) => {
      const callbackMap: Record<string, Function> = {};
      socketServiceMock.on.and.callFake((event: string, cb: Function) => {
        callbackMap[event] = cb;
      });

      (service as any).configureStateSocketFeatures();

      callbackMap['stateUpdate']({} as any);

      service.getLimo1State$().subscribe((state) => {
        expect(state).toBe('En attente');
      });

      service.getLimo2State$().subscribe((state) => {
        expect(state).toBe('En attente');
        done();
      });
    });
  });

  describe('disconnect', () => {
    it('should call socketService.disconnect', () => {
      service.disconnect();
      expect(socketServiceMock.disconnect).toHaveBeenCalled();
    });
  });

  describe('ngOnDestroy', () => {
    it('should call disconnect on destroy', () => {
      spyOn(service, 'disconnect');
      service.ngOnDestroy();
      expect(service.disconnect).toHaveBeenCalled();
    });
  });
});
