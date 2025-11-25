import { Component } from '@angular/core';
import { ComponentFixture, TestBed, fakeAsync, tick } from '@angular/core/testing';
import { Router, RouterOutlet } from '@angular/router';
import { AppComponent } from './app.component';
import { MissionModeService } from '@app/services/mission-mode/mission-mode.service';
import { MissionSessionService } from '@app/services/mission-session/mission-session.service';
import { TelemetryLoggingService } from '@app/services/telemetry-logging/telemetry-logging.service';
import { Subject } from 'rxjs';

type MissionMode = 'REAL' | 'SIMULATION' | null | undefined;

@Component({
  selector: 'app-global-navbar',
  standalone: true,
  template: ''
})
class GlobalNavbarStubComponent {}

describe('AppComponent', () => {
  let fixture: ComponentFixture<AppComponent>;
  let routerSpy: jasmine.SpyObj<Router>;
  let missionModeSpy: jasmine.SpyObj<MissionModeService>;
  let missionSessionSpy: jasmine.SpyObj<MissionSessionService>;
  let telemetryLoggingSpy: jasmine.SpyObj<TelemetryLoggingService>;
  let modeSubject: Subject<MissionMode>;
  let currentUrl = '/home';

  const setRouterUrl = (url: string) => {
    currentUrl = url;
  };

  beforeEach(async () => {
    modeSubject = new Subject<MissionMode>();

    routerSpy = jasmine.createSpyObj('Router', ['navigate']);
    routerSpy.navigate.and.returnValue(Promise.resolve(true));
    Object.defineProperty(routerSpy, 'url', { get: () => currentUrl });

    missionModeSpy = jasmine.createSpyObj(
      'MissionModeService',
      ['startPolling', 'ensureModeLoaded'],
      { mode$: modeSubject.asObservable() }
    );
    missionModeSpy.ensureModeLoaded.and.returnValue(Promise.resolve(null));

    missionSessionSpy = jasmine.createSpyObj('MissionSessionService', ['rehydrateActiveMission']);
    telemetryLoggingSpy = jasmine.createSpyObj('TelemetryLoggingService', ['start']);

    await TestBed.configureTestingModule({
      imports: [AppComponent],
      providers: [
        { provide: Router, useValue: routerSpy },
        { provide: MissionModeService, useValue: missionModeSpy },
        { provide: MissionSessionService, useValue: missionSessionSpy },
        { provide: TelemetryLoggingService, useValue: telemetryLoggingSpy }
      ]
    })
      .overrideComponent(AppComponent, {
        set: {
          imports: [RouterOutlet, GlobalNavbarStubComponent]
        }
      })
      .compileComponents();

    fixture = TestBed.createComponent(AppComponent);
  });

  it('devrait démarrer la télémétrie et le polling lors de ngOnInit', fakeAsync(() => {
    fixture.detectChanges();
    tick();

    expect(telemetryLoggingSpy.start).toHaveBeenCalledTimes(1);
    expect(missionModeSpy.startPolling).toHaveBeenCalledTimes(1);
  }));

  it('rehydrate et redirige vers simulation quand ensureModeLoaded retourne SIMULATION', fakeAsync(() => {
    missionModeSpy.ensureModeLoaded.and.returnValue(Promise.resolve('SIMULATION'));
    setRouterUrl('/home');

    fixture.detectChanges();
    tick();

    expect(missionSessionSpy.rehydrateActiveMission).toHaveBeenCalledTimes(1);
    expect(routerSpy.navigate).toHaveBeenCalledWith(['/simulation-mode']);
  }));

  it('redirige vers real-mode lorsqu’un mode REAL est émis', fakeAsync(() => {
    setRouterUrl('/home');

    fixture.detectChanges();
    tick();

    missionSessionSpy.rehydrateActiveMission.calls.reset();
    routerSpy.navigate.calls.reset();

    modeSubject.next('REAL');
    tick();

    expect(missionSessionSpy.rehydrateActiveMission).toHaveBeenCalledTimes(1);
    expect(routerSpy.navigate).toHaveBeenCalledWith(['/real-mode']);
  }));

  it('retourne à /home lorsque le mode devient null sur une route mission', fakeAsync(() => {
    setRouterUrl('/simulation-mode');

    fixture.detectChanges();
    tick();

    missionSessionSpy.rehydrateActiveMission.calls.reset();
    routerSpy.navigate.calls.reset();

    modeSubject.next(null);
    tick();

    expect(missionSessionSpy.rehydrateActiveMission).not.toHaveBeenCalled();
    expect(routerSpy.navigate).toHaveBeenCalledWith(['/home']);
  }));
});
