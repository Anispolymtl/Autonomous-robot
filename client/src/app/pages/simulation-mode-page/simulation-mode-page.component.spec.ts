import { ComponentFixture, TestBed, fakeAsync, tick } from '@angular/core/testing';
import { SimulationPageComponent } from './simulation-mode-page.component';
import { Router } from '@angular/router';
import { MissionService } from '@app/services/mission.service';
import { MissionSessionService } from '@app/services/mission-session.service';
import { MissionDatabaseService } from '@app/services/mission-database/mission-database.service';
import { of, throwError } from 'rxjs';
import { HttpErrorResponse } from '@angular/common/http';

interface Mission {
  missionName: string;
  robots: string[];
  mode: 'SIMULATION' | 'REAL';
  distance: number;
  durationSec: number;
  status: string;
  logs: any[];
}

describe('SimulationPageComponent', () => {
  let component: SimulationPageComponent;
  let fixture: ComponentFixture<SimulationPageComponent>;
  let routerSpy: jasmine.SpyObj<Router>;
  let missionServiceSpy: jasmine.SpyObj<MissionService>;
  let missionSessionSpy: jasmine.SpyObj<MissionSessionService>;
  let missionDatabaseSpy: jasmine.SpyObj<MissionDatabaseService>;

  const mockMission: Mission = {
    missionName: 'Test Mission',
    robots: [],
    mode: 'SIMULATION',
    distance: 0,
    durationSec: 0,
    status: 'COMPLETED',
    logs: []
  };

  beforeEach(async () => {
    routerSpy = jasmine.createSpyObj('Router', ['navigate']);
    missionServiceSpy = jasmine.createSpyObj('MissionService', ['startMission', 'cancelMission']);
    missionSessionSpy = jasmine.createSpyObj('MissionSessionService', ['rehydrateActiveMission', 'markMissionStarted', 'completeMission']);
    missionDatabaseSpy = jasmine.createSpyObj('MissionDatabaseService', ['createMission']);

    await TestBed.configureTestingModule({
      imports: [SimulationPageComponent],
    })
      .overrideProvider(Router, { useValue: routerSpy })
      .overrideProvider(MissionService, { useValue: missionServiceSpy })
      .overrideProvider(MissionSessionService, { useValue: missionSessionSpy })
      .overrideProvider(MissionDatabaseService, { useValue: missionDatabaseSpy })
      .compileComponents();
  });

  beforeEach(() => {
    fixture = TestBed.createComponent(SimulationPageComponent);
    component = fixture.componentInstance;
    fixture.detectChanges();
  });

  it('should create', () => {
    expect(component).toBeTruthy();
  });

  it('should call rehydrateActiveMission on init', () => {
    expect(missionSessionSpy.rehydrateActiveMission).toHaveBeenCalled();
  });

  describe('setSelectedRobot', () => {
    it('should set the selectedRobotId', () => {
      component.setSelectedRobot('limo2');
      expect(component.selectedRobotId).toBe('limo2');
    });
  });

  describe('back', () => {
    it('should navigate to /home', () => {
      component.back();
      expect(routerSpy.navigate).toHaveBeenCalledWith(['/home']);
    });
  });

  describe('startMission', () => {
    it('should set message, mark mission started and call startMission', () => {
      missionServiceSpy.startMission.and.returnValue(of({}));
      component.startMission();
      expect(component.message).toBe('Mission demandée.');
      expect(missionSessionSpy.markMissionStarted).toHaveBeenCalled();
      expect(missionServiceSpy.startMission).toHaveBeenCalled();
    });

    it('should handle startMission error', () => {
      const error = new HttpErrorResponse({ error: 'fail', status: 500 });
      missionServiceSpy.startMission.and.returnValue(throwError(() => error));
      spyOn(console, 'error');
      component.startMission();
      expect(component.message).toBe('Mission demandée.');
      expect(console.error).toHaveBeenCalledWith('Error starting mission:', error);
    });
  });

  describe('stopMission', () => {
    it('should set message and call cancelMission then finalizeMission on success', fakeAsync(() => {
      missionServiceSpy.cancelMission.and.returnValue(of({}));
      missionSessionSpy.completeMission.and.returnValue(Promise.resolve(mockMission));
      missionDatabaseSpy.createMission.and.returnValue(of(mockMission));

      spyOn(console, 'log');
      component.stopMission();
      tick();

      expect(component.message).toBe('Mission terminée.');
      expect(missionServiceSpy.cancelMission).toHaveBeenCalled();
    }));

    it('should call finalizeMission even if cancelMission errors', fakeAsync(() => {
      const error = new HttpErrorResponse({ error: 'fail', status: 500 });
      missionServiceSpy.cancelMission.and.returnValue(throwError(() => error));
      missionSessionSpy.completeMission.and.returnValue(Promise.resolve(mockMission));
      missionDatabaseSpy.createMission.and.returnValue(of(mockMission));

      spyOn(console, 'error');
      component.stopMission();
      tick();

      expect(console.error).toHaveBeenCalledWith('Error stopping mission:', error);
    }));
  });

  describe('finalizeMission', () => {
    it('should navigate to /home if mission is null', fakeAsync(() => {
      missionSessionSpy.completeMission.and.returnValue(Promise.resolve(null));
      component['finalizeMission']();
      tick();
      expect(routerSpy.navigate).toHaveBeenCalledWith(['/home']);
    }));

    it('should create mission in database and navigate to /home', fakeAsync(() => {
      missionSessionSpy.completeMission.and.returnValue(Promise.resolve(mockMission));
      missionDatabaseSpy.createMission.and.returnValue(of(mockMission));

      component['finalizeMission']();
      tick();

      expect(missionDatabaseSpy.createMission).toHaveBeenCalledWith(mockMission);
      expect(routerSpy.navigate).toHaveBeenCalledWith(['/home']);
    }));

    it('should navigate to /home if createMission errors', fakeAsync(() => {
      missionSessionSpy.completeMission.and.returnValue(Promise.resolve(mockMission));
      missionDatabaseSpy.createMission.and.returnValue(throwError(() => new Error('DB Error')));
      spyOn(console, 'error');

      component['finalizeMission']();
      tick();

      expect(console.error).toHaveBeenCalledWith('Erreur lors de la sauvegarde de la mission:', jasmine.any(Error));
      expect(routerSpy.navigate).toHaveBeenCalledWith(['/home']);
    }));

    it('should navigate to /home if completeMission rejects', fakeAsync(() => {
      missionSessionSpy.completeMission.and.returnValue(Promise.reject(new Error('fail')));
      spyOn(console, 'error');

      component['finalizeMission']();
      tick();

      expect(console.error).toHaveBeenCalledWith('Erreur lors de la finalisation de la mission:', jasmine.any(Error));
      expect(routerSpy.navigate).toHaveBeenCalledWith(['/home']);
    }));
  });
});
