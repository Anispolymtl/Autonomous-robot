import { ComponentFixture, TestBed, fakeAsync, tick } from '@angular/core/testing';
import { of, throwError } from 'rxjs';
import { Router } from '@angular/router';
import { HttpErrorResponse } from '@angular/common/http';
import { RealPageComponent } from './real-mode-page.component';
import { IdentifyService } from '@app/services/identify/identify.service';
import { MissionService } from '@app/services/mission/mission.service';
import { MissionSessionService } from '@app/services/mission-session/mission-session.service';
import { MissionDatabaseService } from '@app/services/mission-database/mission-database.service';
import { MatDialog } from '@angular/material/dialog';

const mockRouter = {
  navigate: jasmine.createSpy('navigate')
};

const mockIdentifyService = {
  identifyRobot: jasmine.createSpy('identifyRobot')
};

const mockMissionService = {
  startMission: jasmine.createSpy('startMission'),
  cancelMission: jasmine.createSpy('cancelMission')
};

const mockMissionSessionService = {
  rehydrateActiveMission: jasmine.createSpy('rehydrateActiveMission'),
  markMissionStarted: jasmine.createSpy('markMissionStarted'),
  appendLog: jasmine.createSpy('appendLog'),
  completeMission: jasmine.createSpy('completeMission')
};

const mockMissionDatabaseService = {
  createMission: jasmine.createSpy('createMission')
};

const mockDialog = {
  open: jasmine.createSpy('open')
};

describe('RealPageComponent', () => {
  let component: RealPageComponent;
  let fixture: ComponentFixture<RealPageComponent>;

  beforeEach(async () => {
    mockDialog.open.and.returnValue({
      afterClosed: () => of(true)
    } as any);

    await TestBed.configureTestingModule({
      imports: [RealPageComponent],
      providers: [
        { provide: Router, useValue: mockRouter },
        { provide: IdentifyService, useValue: mockIdentifyService },
        { provide: MissionService, useValue: mockMissionService },
        { provide: MissionSessionService, useValue: mockMissionSessionService },
        { provide: MissionDatabaseService, useValue: mockMissionDatabaseService },
        { provide: MatDialog, useValue: mockDialog }
      ]
    }).compileComponents();

    fixture = TestBed.createComponent(RealPageComponent);
    component = fixture.componentInstance;
    fixture.detectChanges();
  });

  it('should create', () => {
    expect(component).toBeTruthy();
  });

  it('should call rehydrateActiveMission on init', () => {
    component.ngOnInit();
    expect(mockMissionSessionService.rehydrateActiveMission).toHaveBeenCalled();
  });

  describe('onIdentify', () => {
    it('should handle successful identify', () => {
      const response = { message: 'Identifié' };
      mockIdentifyService.identifyRobot.and.returnValue(of(response));

      component.onIdentify(1);

      expect(mockIdentifyService.identifyRobot).toHaveBeenCalledWith(1);
      expect(component.message).toContain('Réponse du robot 1');
      expect(mockMissionSessionService.appendLog).toHaveBeenCalledWith(jasmine.objectContaining({
        category: 'Command',
        details: jasmine.objectContaining({ success: true })
      }));
    });

    it('should handle identify error', () => {
      const error = new HttpErrorResponse({ status: 500, statusText: 'Server Error' });
      mockIdentifyService.identifyRobot.and.returnValue(throwError(() => error));

      component.onIdentify(2);

      expect(mockIdentifyService.identifyRobot).toHaveBeenCalledWith(2);
      expect(component.message).toContain('Erreur lors de l\'identification');
      expect(mockMissionSessionService.appendLog).toHaveBeenCalledWith(jasmine.objectContaining({
        details: jasmine.objectContaining({ success: false })
      }));
    });
  });

  describe('startMission', () => {
    it('should start mission and mark as started', () => {
      mockMissionService.startMission.and.returnValue(of({ ok: true }));

      component.startMission();

      expect(mockMissionSessionService.markMissionStarted).toHaveBeenCalled();
      expect(mockMissionService.startMission).toHaveBeenCalled();
      expect(component.message).toBe('Mission demandée.');
    });

    it('should handle mission start error', () => {
      const error = new HttpErrorResponse({ status: 500 });
      mockMissionService.startMission.and.returnValue(throwError(() => error));

      component.startMission();

      expect(mockMissionService.startMission).toHaveBeenCalled();
    });
  });

  describe('stopMission', () => {
    it('should stop mission successfully', () => {
      spyOn<any>(component, 'finalizeMission');
      mockMissionService.cancelMission.and.returnValue(of({ ok: true }));

      component.stopMission();

      expect(mockDialog.open).toHaveBeenCalled();
      expect(component['finalizeMission']).toHaveBeenCalled();
      expect(component.message).toBe('Mission terminée.');
    });

    it('should handle mission stop error and still finalize', () => {
      spyOn<any>(component, 'finalizeMission');
      const error = new HttpErrorResponse({ status: 400 });
      mockMissionService.cancelMission.and.returnValue(throwError(() => error));

      component.stopMission();

      expect(mockDialog.open).toHaveBeenCalled();
      expect(component['finalizeMission']).toHaveBeenCalled();
    });
  });

  it('should set selected robot', () => {
    component.setSelectedRobot('limo2');
    expect(component.selectedRobotId).toBe('limo2');
  });

  describe('asRobotNamespace', () => {
    it('should return limo1 for 1', () => {
      expect((component as any).asRobotNamespace(1)).toBe('limo1');
    });

    it('should return limo2 for 2', () => {
      expect((component as any).asRobotNamespace(2)).toBe('limo2');
    });
  });

  describe('finalizeMission', () => {
    it('should navigate home if mission is null', fakeAsync(() => {
      mockMissionSessionService.completeMission.and.returnValue(Promise.resolve(null));

      (component as any).finalizeMission();
      tick();

      expect(mockRouter.navigate).toHaveBeenCalledWith(['/home']);
    }));

    it('should create mission and navigate home on success', fakeAsync(() => {
      const mission = {
        missionName: 'Test',
        robots: ['limo1'],
        mode: 'REAL',
        distance: 10,
        durationSec: 60,
        status: 'completed',
        logs: []
      };
      mockMissionSessionService.completeMission.and.returnValue(Promise.resolve(mission));
      mockMissionDatabaseService.createMission.and.returnValue(of({}));

      (component as any).finalizeMission();
      tick();

      expect(mockMissionDatabaseService.createMission).toHaveBeenCalledWith(jasmine.objectContaining({
        missionName: 'Test'
      }));
      expect(mockRouter.navigate).toHaveBeenCalledWith(['/home']);
    }));

    it('should handle database save error gracefully', fakeAsync(() => {
      const mission = {
        missionName: 'Erreur',
        robots: ['limo1'],
        mode: 'REAL',
        status: 'failed'
      };
      mockMissionSessionService.completeMission.and.returnValue(Promise.resolve(mission));
      mockMissionDatabaseService.createMission.and.returnValue(throwError(() => new Error('DB Error')));

      (component as any).finalizeMission();
      tick();

      expect(mockRouter.navigate).toHaveBeenCalledWith(['/home']);
    }));

    it('should handle promise rejection', fakeAsync(() => {
      mockMissionSessionService.completeMission.and.returnValue(Promise.reject('error'));
      (component as any).finalizeMission();
      tick();
      expect(mockRouter.navigate).toHaveBeenCalledWith(['/home']);
    }));
  });
});
