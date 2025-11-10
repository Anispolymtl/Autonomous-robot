import { ComponentFixture, TestBed, fakeAsync, tick } from '@angular/core/testing';
import { RealPageComponent } from './real-mode-page.component';
import { Router } from '@angular/router';
import { IdentifyService } from '@app/services/identify.service';
import { MissionService } from '@app/services/mission.service';
import { MissionSessionService } from '@app/services/mission-session.service';
import { MissionDatabaseService } from '@app/services/mission-database/mission-database.service';
import { Component } from '@angular/core';
import { of, throwError } from 'rxjs';
import { HttpErrorResponse } from '@angular/common/http';

@Component({ selector: 'app-map', template: '' })
class MapStubComponent {}

@Component({ selector: 'app-robot-status', template: '' })
class RobotStatusStubComponent {}

describe('RealPageComponent', () => {
  let component: RealPageComponent;
  let fixture: ComponentFixture<RealPageComponent>;
  let routerSpy: jasmine.SpyObj<Router>;
  let identifyServiceSpy: jasmine.SpyObj<IdentifyService>;
  let missionServiceSpy: jasmine.SpyObj<MissionService>;
  let missionSessionSpy: jasmine.SpyObj<MissionSessionService>;
  let missionDatabaseSpy: jasmine.SpyObj<MissionDatabaseService>;

  beforeEach(async () => {
    routerSpy = jasmine.createSpyObj('Router', ['navigate']);
    identifyServiceSpy = jasmine.createSpyObj('IdentifyService', ['identifyRobot']);
    missionServiceSpy = jasmine.createSpyObj('MissionService', ['startMission', 'cancelMission']);
    missionSessionSpy = jasmine.createSpyObj('MissionSessionService', ['rehydrateActiveMission', 'markMissionStarted', 'completeMission']);
    missionDatabaseSpy = jasmine.createSpyObj('MissionDatabaseService', ['createMission']);

    await TestBed.configureTestingModule({
      imports: [RealPageComponent],
    })
    .overrideComponent(RealPageComponent, {
      set: {
        imports: [MapStubComponent, RobotStatusStubComponent]
      }
    })
    .overrideProvider(Router, { useValue: routerSpy })
    .overrideProvider(IdentifyService, { useValue: identifyServiceSpy })
    .overrideProvider(MissionService, { useValue: missionServiceSpy })
    .overrideProvider(MissionSessionService, { useValue: missionSessionSpy })
    .overrideProvider(MissionDatabaseService, { useValue: missionDatabaseSpy })
    .compileComponents();
  });

  beforeEach(() => {
    fixture = TestBed.createComponent(RealPageComponent);
    component = fixture.componentInstance;
    fixture.detectChanges();
  });

  it('should create', () => {
    expect(component).toBeTruthy();
  });

  it('should call rehydrateActiveMission on init', () => {
    expect(missionSessionSpy.rehydrateActiveMission).toHaveBeenCalled();
  });

  describe('onIdentify', () => {
    it('should set message on success', () => {
      identifyServiceSpy.identifyRobot.and.returnValue(of({ message: 'ok' }));
      component.onIdentify(1);
      expect(component.message).toBe('Réponse du robot 1 : ok');
    });

    it('should set message on error', () => {
      const error = new HttpErrorResponse({ error: 'fail', status: 500 });
      identifyServiceSpy.identifyRobot.and.returnValue(throwError(() => error));
      component.onIdentify(2);
      expect(component.message).toContain('Erreur lors de l\'identification du robot 2');
    });
  });

  describe('startMission', () => {
    it('should mark mission started and call startMission', () => {
      missionServiceSpy.startMission.and.returnValue(of({}));
      component.startMission();
      expect(component.message).toBe('Mission demandée.');
      expect(missionSessionSpy.markMissionStarted).toHaveBeenCalled();
      expect(missionServiceSpy.startMission).toHaveBeenCalled();
    });
  });

  describe('stopMission', () => {
    it('should set message and call cancelMission', () => {
      missionServiceSpy.cancelMission.and.returnValue(of({}));
      spyOn<any>(component, 'finalizeMission');
      component.stopMission();
      expect(component.message).toBe('Mission terminée.');
      expect(missionServiceSpy.cancelMission).toHaveBeenCalled();
      expect((component as any).finalizeMission).toHaveBeenCalled();
    });
  });

  it('should set selected robot', () => {
    component.setSelectedRobot('limo2');
    expect(component.selectedRobotId).toBe('limo2');
  });

  it('should navigate back', () => {
    component.back();
    expect(routerSpy.navigate).toHaveBeenCalledWith(['/home']);
  });

  describe('finalizeMission', () => {
    const mockMission = {
      missionName: 'Test Mission',
      robots: [],
      mode: 'REAL' as const,
      status: 'done',
      distance: 100,
      durationSec: 3600,
      logs: []
    };

    it('should complete mission and create mission in db', fakeAsync(() => {
      missionSessionSpy.completeMission.and.returnValue(Promise.resolve(mockMission));
      missionDatabaseSpy.createMission.and.returnValue(of(mockMission));

      (component as any).finalizeMission();
      tick();

      expect(missionSessionSpy.completeMission).toHaveBeenCalled();
      expect(missionDatabaseSpy.createMission).toHaveBeenCalledWith(jasmine.objectContaining({
        missionName: 'Test Mission'
      }));
      expect(routerSpy.navigate).toHaveBeenCalledWith(['/home']);
    }));

    it('should navigate home if no mission', fakeAsync(() => {
      missionSessionSpy.completeMission.and.returnValue(Promise.resolve(null));
      (component as any).finalizeMission();
      tick();
      expect(routerSpy.navigate).toHaveBeenCalledWith(['/home']);
    }));

    it('should navigate home on error', fakeAsync(() => {
      missionSessionSpy.completeMission.and.returnValue(Promise.reject('fail'));
      (component as any).finalizeMission();
      tick();
      expect(routerSpy.navigate).toHaveBeenCalledWith(['/home']);
    }));
  });
});
