import { TestBed } from '@angular/core/testing';
import { TelemetryLoggingService } from './telemetry-logging.service';
import { MissionSessionService } from '@app/services/mission-session/mission-session.service';
import { SocketService } from '@app/services/socket/socket.service';
import { PoseData } from '@app/interfaces/pose-data';
import { Socket } from 'socket.io-client';

describe('TelemetryLoggingService', () => {
  let service: TelemetryLoggingService;
  let missionSessionService: jasmine.SpyObj<MissionSessionService>;
  let socketService: jasmine.SpyObj<SocketService>;
  let missionActive = false;
  let currentSocket: Socket | undefined;

  const createPose = (x: number, y: number, z: number): PoseData => ({
    header: { frame_id: 'map' },
    pose: {
      position: { x, y, z },
      orientation: { x: 0, y: 0, z: 0, w: 1 }
    }
  });

  const createSocketMock = () =>
    ({
      on: jasmine.createSpy('on'),
      off: jasmine.createSpy('off')
    }) as unknown as Socket;

  beforeEach(() => {
    missionActive = false;
    currentSocket = undefined;

    missionSessionService = jasmine.createSpyObj('MissionSessionService', ['appendLog', 'updateRobotDistance']);
    Object.defineProperty(missionSessionService, 'hasActiveMission', {
      get: () => missionActive
    });

    socketService = jasmine.createSpyObj('SocketService', ['isSocketAlive', 'connect']);
    socketService.isSocketAlive.and.returnValue(true);
    Object.defineProperty(socketService, 'getSocket', {
      get: () => currentSocket
    });

    TestBed.configureTestingModule({
      providers: [
        TelemetryLoggingService,
        { provide: MissionSessionService, useValue: missionSessionService },
        { provide: SocketService, useValue: socketService }
      ]
    });

    service = TestBed.inject(TelemetryLoggingService);
  });

  const setMissionState = (isActive: boolean) => {
    missionActive = isActive;
  };

  it('devrait être créé', () => {
    expect(service).toBeTruthy();
  });

  it('tick réinitialise la télémétrie lorsque la mission démarre', () => {
    const resetSpy = spyOn<any>(service, 'resetTelemetry');
    setMissionState(true);

    service['tick']();

    expect(resetSpy).toHaveBeenCalledTimes(1);
  });

  it('tick ne collecte pas de données sans mission active', () => {
    const ensureSpy = spyOn<any>(service, 'ensureSocketListeners');
    const flushSpy = spyOn<any>(service, 'flushSensorLogs');

    setMissionState(false);
    service['tick']();

    expect(ensureSpy).not.toHaveBeenCalled();
    expect(flushSpy).not.toHaveBeenCalled();
  });

  it('tick déclenche la collecte lorsque la mission est active', () => {
    const ensureSpy = spyOn<any>(service, 'ensureSocketListeners');
    const flushSpy = spyOn<any>(service, 'flushSensorLogs');

    setMissionState(true);
    service['tick']();

    expect(ensureSpy).toHaveBeenCalledTimes(1);
    expect(flushSpy).toHaveBeenCalledTimes(1);
  });

  it('handlePoseUpdate cumule la distance et met en cache la pose', () => {
    setMissionState(true);
    service['poseCache'].limo1 = createPose(0, 0, 0);
    service['totalDistance'].limo1 = 0;

    const nextPose = createPose(3, 4, 0);
    service['handlePoseUpdate']({ robot: 'limo1', poseData: nextPose });

    expect(service['poseCache'].limo1).toEqual(nextPose);
    expect(service['totalDistance'].limo1).toBeCloseTo(5, 5);
  });

  it("handlePoseUpdate ignore les mises à jour lorsqu'il n'y a pas de mission active", () => {
    setMissionState(false);

    service['handlePoseUpdate']({ robot: 'limo1', poseData: createPose(1, 1, 1) });

    expect(service['poseCache'].limo1).toBeNull();
  });

  it('flushSensorLogs écrit un log et met à jour la distance', () => {
    const pose = createPose(1.2345, 2, 0.5);
    service['poseCache'].limo1 = pose;
    service['lastLoggedAt'].limo1 = 0;
    service['totalDistance'].limo1 = 7.89123;
    spyOn(Date, 'now').and.returnValue(2500);

    service['flushSensorLogs']();

    expect(missionSessionService.appendLog).toHaveBeenCalledTimes(1);
    const logPayload = missionSessionService.appendLog.calls.mostRecent().args[0];
    expect(logPayload.robot).toBe('limo1');
    expect(logPayload.category).toBe('Sensor');
    expect(logPayload.details).toBeDefined();
    const details = logPayload.details as { totalDistance: number; posX: number };
    expect(details.totalDistance).toBe(7.891);
    expect(details.posX).toBe(1.234);
    expect(missionSessionService.updateRobotDistance).toHaveBeenCalledWith('limo1', 7.891);
    expect(service['lastLoggedAt'].limo1).toBe(2500);
  });

  it("flushSensorLogs respecte l'intervalle minimal entre deux écritures", () => {
    service['poseCache'].limo1 = createPose(0, 0, 0);
    service['lastLoggedAt'].limo1 = 2600;
    spyOn(Date, 'now').and.returnValue(3000);

    service['flushSensorLogs']();

    expect(missionSessionService.appendLog).not.toHaveBeenCalled();
  });

  it("ensureSocketListeners connecte le socket et s'attache aux événements", () => {
    socketService.isSocketAlive.and.returnValue(false);
    currentSocket = createSocketMock();

    service['ensureSocketListeners']();

    expect(socketService.connect).toHaveBeenCalledWith('client');
    expect(currentSocket?.on).toHaveBeenCalledWith('poseUpdate', jasmine.any(Function));
    expect(currentSocket?.on).toHaveBeenCalledWith('disconnect', jasmine.any(Function));
  });

  it("ensureSocketListeners détache les anciens écouteurs lorsqu'un nouveau socket est utilisé", () => {
    const firstSocket = createSocketMock();
    const secondSocket = createSocketMock();

    currentSocket = firstSocket;
    service['ensureSocketListeners']();

    currentSocket = secondSocket;
    service['ensureSocketListeners']();

    expect(firstSocket.off).toHaveBeenCalledWith('poseUpdate', jasmine.any(Function));
    expect(firstSocket.off).toHaveBeenCalledWith('disconnect', jasmine.any(Function));
    expect(secondSocket.on).toHaveBeenCalledWith('poseUpdate', jasmine.any(Function));
    expect(secondSocket.on).toHaveBeenCalledWith('disconnect', jasmine.any(Function));
  });
});
