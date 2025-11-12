import { ComponentFixture, TestBed, fakeAsync, tick } from '@angular/core/testing';
import { ActivatedRoute } from '@angular/router';
import { Subject, of } from 'rxjs';
import { LogsPageComponent } from './logs-page.component';
import { ActiveMissionResponse, MissionModeService } from '@app/services/mission-mode/mission-mode.service';
import { SocketService } from '@app/services/socket/socket.service';
import { MissionLogEntry } from '@app/interfaces/mission';

type Handler = (payload?: unknown) => void;

describe('LogsPageComponent', () => {
  let component: LogsPageComponent;
  let fixture: ComponentFixture<LogsPageComponent>;
  let missionModeServiceSpy: jasmine.SpyObj<MissionModeService>;
  let socketServiceSpy: jasmine.SpyObj<SocketService>;
  let queryParamMapSubject: Subject<any>;
  let socketHandlers: Record<string, Handler[]>;
  let onceHandlers: Record<string, Array<() => void>>;

  const emitOnce = (event: string) => {
    (onceHandlers[event] || []).forEach((handler) => handler());
    onceHandlers[event] = [];
  };

  const emitQueryParams = (params: Record<string, string | null>) => {
    queryParamMapSubject.next({
      get: (key: string) => params[key] ?? null
    });
  };

  const createLog = (timestamp: string, action: string): MissionLogEntry => ({
    timestamp,
    robot: 'limo1',
    category: 'Command',
    action,
    details: {}
  });

  beforeEach(async () => {
    queryParamMapSubject = new Subject();
    missionModeServiceSpy = jasmine.createSpyObj('MissionModeService', ['fetchActiveMission']);
    missionModeServiceSpy.fetchActiveMission.and.returnValue(of(null));

    socketServiceSpy = jasmine.createSpyObj('SocketService', ['connect', 'isSocketAlive', 'on', 'off', 'once']);
    socketServiceSpy.isSocketAlive.and.returnValue(true);
    socketHandlers = {};
    onceHandlers = {};
    socketServiceSpy.on.and.callFake(<T>(event: string, handler: (data: T) => void) => {
      (socketHandlers[event] ||= []).push(handler as Handler);
    });
    socketServiceSpy.off.and.callFake((event: string, handler: Handler) => {
      socketHandlers[event] = (socketHandlers[event] || []).filter((registered) => registered !== handler);
    });
    socketServiceSpy.once.and.callFake((event: string, handler: () => void) => {
      (onceHandlers[event] ||= []).push(handler);
    });

    await TestBed.configureTestingModule({
      imports: [LogsPageComponent],
      providers: [
        { provide: MissionModeService, useValue: missionModeServiceSpy },
        { provide: SocketService, useValue: socketServiceSpy },
        { provide: ActivatedRoute, useValue: { queryParamMap: queryParamMapSubject.asObservable() } }
      ]
    }).compileComponents();

    fixture = TestBed.createComponent(LogsPageComponent);
    component = fixture.componentInstance;
  });

  it('crée le composant', () => {
    expect(component).toBeTruthy();
  });

  it('initialise missionId et missionName à partir des query params', fakeAsync(() => {
    const mission: ActiveMissionResponse = {
      missionId: 'abc',
      missionName: 'MissionNameTest',
      mode: 'REAL',
      durationSec: 0,
      distance: 0,
      robots: ['limo1'],
      logs: [],
      status: 'RUNNING'
    };
    missionModeServiceSpy.fetchActiveMission.and.returnValue(of(mission));

    fixture.detectChanges();
    emitQueryParams({ missionId: 'abc', missionName: 'MissionNameTest' });
    tick();
    expect(component.missionId).toBe('abc');
    expect(component.missionName).toBe('MissionNameTest');
  }));

  it('charge la mission active et enregistre les écouteurs sockets', fakeAsync(() => {
    const mission: ActiveMissionResponse = {
      missionId: 'mid',
      missionName: 'Mission Alpha',
      mode: 'REAL',
      durationSec: 0,
      distance: 5,
      robots: ['limo1'],
      status: 'RUNNING',
      logs: [createLog('2025-01-01T10:00:00Z', 'older'), createLog('2025-01-01T10:05:00Z', 'newer')]
    };
    missionModeServiceSpy.fetchActiveMission.and.returnValue(of(mission));

    fixture.detectChanges();
    tick();

    expect(component.missionId).toBe('mid');
    expect(component.liveData[0].action).toBe('newer');
    expect(socketServiceSpy.on).toHaveBeenCalledWith('mission:updated', jasmine.any(Function));
    expect(socketServiceSpy.on).toHaveBeenCalledWith('mission:finalized', jasmine.any(Function));
    expect(component['socketListenersRegistered']).toBeTrue();
  }));

  it('réinitialise la session quand aucune mission active n’est trouvée', fakeAsync(() => {
    const cleanupSpy = spyOn<any>(component, 'cleanupSocketListeners').and.callThrough();
    component['missionId'] = 'old';
    component['missionName'] = 'Old mission';
    component['liveData'] = [createLog('2025-01-01T10:00:00Z', 'action')];

    missionModeServiceSpy.fetchActiveMission.and.returnValue(of(null));
    (component as any).loadActiveMission();
    tick();

    expect(component.missionId).toBeNull();
    expect(component.missionName).toBeNull();
    expect(component.liveData).toEqual([]);
    expect(cleanupSpy).toHaveBeenCalled();
  }));

  it('missionUpdateHandler remplace les logs quand la mission correspond', () => {
    component['missionId'] = 'abc';
    component['missionName'] = 'Initial';
    const handler = component['missionUpdateHandler'] as Handler;

    handler({
      missionId: 'abc',
      mission: {
        missionName: 'Updated',
        logs: [createLog('2025-01-01T11:00:00Z', 'late'), createLog('2025-01-01T10:00:00Z', 'early')]
      }
    });

    expect(component.liveData[0].action).toBe('late');
    expect(component.missionName).toBe('Updated');
  });

  it('missionFinalizedHandler efface la mission et nettoie les sockets', () => {
    component['missionId'] = 'abc';
    component['missionName'] = 'Mission';
    const cleanupSpy = spyOn<any>(component, 'cleanupSocketListeners').and.callThrough();
    const handler = component['missionFinalizedHandler'] as Handler;

    handler({
      missionId: 'abc',
      mission: { logs: [createLog('2025-01-01T11:00:00Z', 'done')] }
    });

    expect(component.missionId).toBeNull();
    expect(component.missionName).toBeNull();
    expect(component.liveData.length).toBe(1);
    expect(cleanupSpy).toHaveBeenCalled();
  });

  it('ensureSocketConnected établit la connexion lorsque nécessaire', fakeAsync(() => {
    let resolved = false;
    socketServiceSpy.isSocketAlive.and.returnValue(false);
    component['ensureSocketConnected']().then(() => (resolved = true));

    expect(socketServiceSpy.connect).toHaveBeenCalledWith('client');
    expect(socketServiceSpy.once).toHaveBeenCalledWith('connect', jasmine.any(Function));

    emitOnce('connect');
    tick();

    expect(resolved).toBeTrue();
  }));

  it('bascule entre les onglets', () => {
    component.switchTab('history');
    expect(component.activeTab).toBe('history');
    component.switchTab('live');
    expect(component.activeTab).toBe('live');
  });

  it('calcule correctement hasActiveMission', () => {
    component.missionId = '1';
    expect(component.hasActiveMission).toBeTrue();
    component.missionId = null;
    component.missionName = 'Test';
    expect(component.hasActiveMission).toBeTrue();
    component.missionName = null;
    expect(component.hasActiveMission).toBeFalse();
  });

  it('trie les logs du plus récent au plus ancien', () => {
    const sorted = component['sortLogs']([
      createLog('2025-01-01T10:00:00Z', 'a'),
      createLog('2025-01-01T10:05:00Z', 'b')
    ]);
    expect(sorted[0].action).toBe('b');
    expect(sorted[1].action).toBe('a');
  });

  it('nettoie les listeners lors du destroy', () => {
    component['socketListenersRegistered'] = true;
    component['missionUpdateHandler'] = () => {};
    component['missionFinalizedHandler'] = () => {};

    component.ngOnDestroy();

    expect(socketServiceSpy.off).toHaveBeenCalledWith('mission:updated', component['missionUpdateHandler']);
    expect(socketServiceSpy.off).toHaveBeenCalledWith('mission:finalized', component['missionFinalizedHandler']);
  });
});
