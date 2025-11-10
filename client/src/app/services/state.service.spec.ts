import { TestBed } from '@angular/core/testing';
import { MissionStateService } from './state.service';
import { SocketService } from '@app/services/socket.service';
import { take } from 'rxjs/operators';

describe('MissionStateService', () => {
  let service: MissionStateService;
  let socketServiceSpy: jasmine.SpyObj<SocketService>;

  beforeEach(() => {
    const spy = jasmine.createSpyObj('SocketService', ['isSocketAlive', 'connect', 'disconnect', 'on']);

    TestBed.configureTestingModule({
      providers: [
        MissionStateService,
        { provide: SocketService, useValue: spy },
      ]
    });

    service = TestBed.inject(MissionStateService);
    socketServiceSpy = TestBed.inject(SocketService) as jasmine.SpyObj<SocketService>;
  });

  it('should be created', () => {
    expect(service).toBeTruthy();
  });

  it('should connect to socket if not alive', () => {
    socketServiceSpy.isSocketAlive.and.returnValue(false);
    socketServiceSpy.on.and.callFake(() => {});

    service.connectToSocket();

    expect(socketServiceSpy.connect).toHaveBeenCalledWith('client');
    expect(socketServiceSpy.on).toHaveBeenCalledWith('stateUpdate', jasmine.any(Function));
  });

  it('should not connect to socket if already alive', () => {
    socketServiceSpy.isSocketAlive.and.returnValue(true);

    service.connectToSocket();

    expect(socketServiceSpy.connect).not.toHaveBeenCalled();
  });

  it('should update limo1 state when stateUpdate is received', (done) => {
    socketServiceSpy.isSocketAlive.and.returnValue(false);

    socketServiceSpy.on.and.callFake((event: string, cb: (data: any) => void) => {
      cb({ robot: 'limo1', state: 'En route' });
    });

    service.connectToSocket();

    service.getLimo1State$().pipe(take(1)).subscribe(state => {
      expect(state).toBe('En route');
      done();
    });
  });

  it('should update limo2 state when stateUpdate is received', (done) => {
    socketServiceSpy.isSocketAlive.and.returnValue(false);

    socketServiceSpy.on.and.callFake((event: string, cb: (data: any) => void) => {
      cb({ robot: 'limo2', state: 'Occupé' });
    });

    service.connectToSocket();

    service.getLimo2State$().pipe(take(1)).subscribe(state => {
      expect(state).toBe('Occupé');
      done();
    });
  });

  it('should ignore stateUpdate with missing data', () => {
    socketServiceSpy.isSocketAlive.and.returnValue(false);

    const consoleSpy = spyOn(console, 'log');

    socketServiceSpy.on.and.callFake((event: string, cb: (data: any) => void) => {
      cb({});
    });

    service.connectToSocket();

    expect(consoleSpy).toHaveBeenCalledWith('[STATE] Abort  message');
  });

  it('should call disconnect on destroy', () => {
    service.ngOnDestroy();
    expect(socketServiceSpy.disconnect).toHaveBeenCalled();
  });
});
