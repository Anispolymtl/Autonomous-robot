import { TestBed, fakeAsync, tick } from '@angular/core/testing';
import { of } from 'rxjs';
import { MissionSessionService } from './mission-session.service';
import { SocketService } from '@app/services/socket/socket.service';
import { ActiveMissionResponse, MissionModeService } from '@app/services/mission-mode/mission-mode.service';
import { Mission } from '@app/interfaces/mission';

type SocketHandler = (...args: unknown[]) => void;

describe('MissionSessionService', () => {
  let service: MissionSessionService;
  let socketService: jasmine.SpyObj<SocketService>;
  let missionModeService: jasmine.SpyObj<MissionModeService>;
  let listeners: Record<string, SocketHandler[]>;

  const emit = (event: string, payload?: unknown) => {
    (listeners[event] || []).forEach((handler) => handler(payload));
  };

  const createMission = (overrides: Partial<Mission> = {}): Mission => ({
    missionName: 'Mission Test',
    mode: 'REAL',
    durationSec: 0,
    distance: 0,
    robots: ['limo1'],
    logs: [],
    status: 'PENDING',
    ...overrides
  });

  beforeEach(() => {
    listeners = {};
    socketService = jasmine.createSpyObj('SocketService', ['connect', 'disconnect', 'isSocketAlive', 'on', 'off', 'send']);
    socketService.isSocketAlive.and.returnValue(true);
    socketService.on.and.callFake(<T>(event: string, handler: (data: T) => void) => {
      (listeners[event] ||= []).push(handler as SocketHandler);
    });
    socketService.off.and.callFake(<T>(event: string, handler: (data: T) => void) => {
      listeners[event] = (listeners[event] || []).filter((registered) => registered !== handler);
    });

    missionModeService = jasmine.createSpyObj('MissionModeService', ['setMode', 'fetchActiveMission']);
    missionModeService.fetchActiveMission.and.returnValue(of(null));

    TestBed.configureTestingModule({
      providers: [
        MissionSessionService,
        { provide: SocketService, useValue: socketService },
        { provide: MissionModeService, useValue: missionModeService }
      ]
    });

    service = TestBed.inject(MissionSessionService);
  });

  it('devrait être créé', () => {
    expect(service).toBeTruthy();
  });

  it('initializeMission résout après mission:created et met à jour le mode', fakeAsync(() => {
    spyOn<any>(service, 'ensureSocketConnected').and.returnValue(Promise.resolve());
    const registerSpy = spyOn<any>(service, 'registerMissionListeners').and.callThrough();
    const mission = createMission({ missionName: 'Alpha', mode: 'REAL' });

    let payload: { missionId: string; mission: Mission } | undefined;
    service.initializeMission('Alpha', 'REAL').then((result) => (payload = result));
    tick();

    expect(socketService.send).toHaveBeenCalledWith('mission:create', jasmine.objectContaining({ missionName: 'Alpha' }));

    emit('mission:created', { missionId: 'm-1', mission });
    tick();

    expect(payload?.missionId).toBe('m-1');
    expect(service.currentMission).toEqual(mission);
    expect(registerSpy).toHaveBeenCalled();
    expect(missionModeService.setMode).toHaveBeenCalledWith('REAL');
  }));

  it('initializeMission rejette lorsque mission:error est reçu', fakeAsync(() => {
    spyOn<any>(service, 'ensureSocketConnected').and.returnValue(Promise.resolve());
    let error: Error | undefined;

    service.initializeMission('Bravo', 'SIMULATION').catch((err) => (error = err));
    tick();
    emit('mission:error', { message: 'Impossible de créer' });
    tick();

    expect(error?.message).toBe('Impossible de créer');
  }));

  it('markMissionStarted envoie une mise à jour et un log', () => {
    service['missionId'] = 'active';
    const nowSpy = spyOn(Date, 'now').and.returnValue(10_000);

    service.markMissionStarted();

    expect(socketService.send).toHaveBeenCalledWith('mission:update', {
      missionId: 'active',
      data: jasmine.objectContaining({ status: 'RUNNING', durationSec: 0 })
    });
    expect(socketService.send).toHaveBeenCalledWith('mission:add-log', {
      missionId: 'active',
      log: jasmine.objectContaining({ action: 'mission_started', category: 'Command' })
    });
    expect(service['missionStartTimestamp']).toBe(10_000);
    nowSpy.and.callThrough();
  });

  it('sendMissionUpdate calcule la durée écoulée', () => {
    service['missionId'] = 'active';
    service['missionStartTimestamp'] = 5_000;
    const nowSpy = spyOn(Date, 'now').and.returnValue(7_500);

    service.sendMissionUpdate({ status: 'RUNNING' });

    expect(socketService.send).toHaveBeenCalledWith('mission:update', {
      missionId: 'active',
      data: jasmine.objectContaining({ status: 'RUNNING', durationSec: 3 })
    });
    nowSpy.and.callThrough();
  });

  it('updateRobotDistance additionne les distances et évite les doublons', () => {
    service['missionId'] = 'active';

    service.updateRobotDistance('limo1', 2.5);
    service.updateRobotDistance('limo2', 1.5);

    expect(socketService.send).toHaveBeenCalledWith('mission:update', {
      missionId: 'active',
      data: jasmine.objectContaining({ distance: 4 })
    });

    socketService.send.calls.reset();
    service.updateRobotDistance('limo2', 1.5);
    expect(socketService.send).not.toHaveBeenCalled();
  });

  it('rehydrateActiveMission restaure la mission courante', async () => {
    const nowSpy = spyOn(Date, 'now').and.returnValue(10_000);
    const mission: ActiveMissionResponse = {
      missionId: 'rehydrated',
      missionName: 'Persisted',
      mode: 'SIMULATION',
      durationSec: 8,
      distance: 12,
      robots: ['limo1'],
      logs: [],
      status: 'RUNNING'
    };
    missionModeService.fetchActiveMission.and.returnValue(of(mission));

    await service.rehydrateActiveMission();

    expect(service.currentMission).toEqual(mission);
    expect(service['missionStartTimestamp']).toBe(2_000);
    expect(missionModeService.setMode).toHaveBeenCalledWith('SIMULATION');
    nowSpy.and.callThrough();
  });

  it('appendLog normalise les champs manquants', () => {
    service['missionId'] = 'active';
    service.appendLog({ action: 'custom_action' });

    expect(socketService.send).toHaveBeenCalledWith('mission:add-log', {
      missionId: 'active',
      log: jasmine.objectContaining({
        action: 'custom_action',
        category: 'Command',
        robot: 'limo1'
      })
    });
  });

  it('completeMission finalise la mission et réinitialise la session', fakeAsync(() => {
    const mission = createMission({ distance: 10, status: 'COMPLETED' });
    service['missionId'] = 'active';
    service['missionSnapshot'] = mission;
    service['missionStartTimestamp'] = 1_000;
    socketService.isSocketAlive.and.returnValue(true);
    service['registerMissionListeners']();

    let resolved: Mission | null | undefined;
    service.completeMission().then((result) => (resolved = result));

    expect(socketService.send).toHaveBeenCalledWith('mission:update', jasmine.any(Object));
    expect(socketService.send).toHaveBeenCalledWith('mission:add-log', jasmine.any(Object));
    expect(socketService.send).toHaveBeenCalledWith('mission:complete', { missionId: 'active' });

    emit('mission:finalized', { missionId: 'active', mission });
    tick();

    expect(resolved).toEqual(mission);
    expect(service.hasActiveMission).toBeFalse();
    expect(socketService.disconnect).toHaveBeenCalled();
    expect(missionModeService.setMode).toHaveBeenCalledWith(null);
  }));

  it('ensureSocketConnected établit une connexion quand nécessaire', fakeAsync(() => {
    socketService.isSocketAlive.and.returnValue(false);

    let resolved = false;
    service['ensureSocketConnected']().then(() => (resolved = true));

    expect(socketService.connect).toHaveBeenCalledWith('client');
    emit('connect');
    tick();

    expect(resolved).toBeTrue();
    expect(socketService.off).toHaveBeenCalledWith('connect', jasmine.any(Function));
  }));

  it('ensureSocketConnected rejette sur connect_error', fakeAsync(() => {
    socketService.isSocketAlive.and.returnValue(false);

    let error: Error | undefined;
    service['ensureSocketConnected']().catch((err) => (error = err as Error));

    emit('connect_error', new Error('KO'));
    tick();

    expect(error?.message).toBe('KO');
  }));

  it('registerMissionListeners ne s’abonne qu’une seule fois', () => {
    service['registerMissionListeners']();
    service['registerMissionListeners']();

    const subscriptionCount = socketService.on.calls.all().filter((call) => call.args[0] === 'mission:updated').length;
    expect(subscriptionCount).toBe(1);
  });
});
