import { ComponentFixture, TestBed, fakeAsync, tick } from '@angular/core/testing';
import { LogsPageComponent } from './logs-page.component';
import { ActivatedRoute } from '@angular/router';
import { Subject } from 'rxjs';
import { MissionModeService } from '@app/services/mission-mode/mission-mode.service';
import { SocketService } from '@app/services/socket/socket.service';

describe('LogsPageComponent', () => {
  let component: LogsPageComponent;
  let fixture: ComponentFixture<LogsPageComponent>;
  let missionModeServiceSpy: jasmine.SpyObj<MissionModeService>;
  let socketServiceSpy: jasmine.SpyObj<SocketService>;
  let queryParamMapSubject: Subject<any>;


  beforeEach(async () => {
    queryParamMapSubject = new Subject();

    missionModeServiceSpy = jasmine.createSpyObj('MissionModeService', ['fetchActiveMission']);
    socketServiceSpy = jasmine.createSpyObj('SocketService', ['connect', 'isSocketAlive', 'on', 'off', 'once']);

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

  it('should create the component', () => {
    expect(component).toBeTruthy();
  });

  it('should initialize missionId and missionName from query params', fakeAsync(() => {
    fixture.detectChanges();
    queryParamMapSubject.next({ get: (key: string) => key === 'missionId' ? 'abc' : 'MissionNameTest' });
    tick();
    expect(component.missionId).toBe('abc');
    expect(component.missionName).toBe('MissionNameTest');
  }));

  it('should switch the active tab', () => {
    component.switchTab('history');
    expect(component.activeTab).toBe('history');
    component.switchTab('live');
    expect(component.activeTab).toBe('live');
  });

  it('should correctly return hasActiveMission', () => {
    component.missionId = '1';
    expect(component.hasActiveMission).toBeTrue();
    component.missionId = null;
    component.missionName = 'Test';
    expect(component.hasActiveMission).toBeTrue();
    component.missionName = null;
    expect(component.hasActiveMission).toBeFalse();
  });

  it('should sort logs from newest to oldest', () => {
    const sortedLogs = component['sortLogs']([
      { timestamp: '2025-01-01T12:00:00Z', robot: 'limo1', category: 'Command', action: 'a', details: {} },
      { timestamp: '2025-01-01T12:05:00Z', robot: 'limo1', category: 'Command', action: 'b', details: {} }
    ]);
    expect(sortedLogs[0].action).toBe('b');
    expect(sortedLogs[1].action).toBe('a');
  });

  it('should clean up socket listeners on ngOnDestroy', () => {
    socketServiceSpy.on.and.callFake(() => {});
    component['socketListenersRegistered'] = true;
    component['missionUpdateHandler'] = () => {};
    component['missionFinalizedHandler'] = () => {};
    component.ngOnDestroy();
    expect(socketServiceSpy.off).toHaveBeenCalledWith('mission:updated', component['missionUpdateHandler']);
    expect(socketServiceSpy.off).toHaveBeenCalledWith('mission:finalized', component['missionFinalizedHandler']);
  });
});
