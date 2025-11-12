import { ComponentFixture, TestBed, fakeAsync, tick } from '@angular/core/testing';
import { SimulationPageComponent } from './simulation-mode-page.component';
import { Router } from '@angular/router';
import { of, throwError } from 'rxjs';
import { HttpErrorResponse } from '@angular/common/http';
import { MissionService } from '@app/services/mission.service';
import { MissionSessionService } from '@app/services/mission-session.service';
import { MissionDatabaseService } from '@app/services/mission-database.service';

describe('SimulationPageComponent', () => {
  let component: SimulationPageComponent;
  let fixture: ComponentFixture<SimulationPageComponent>;

  let mockRouter: jasmine.SpyObj<Router>;
  let mockMissionService: jasmine.SpyObj<MissionService>;
  let mockMissionSessionService: jasmine.SpyObj<MissionSessionService>;
  let mockMissionDatabaseService: jasmine.SpyObj<MissionDatabaseService>;

  beforeEach(async () => {
    mockRouter = jasmine.createSpyObj('Router', ['navigate']);
    mockMissionService = jasmine.createSpyObj('MissionService', ['startMission', 'cancelMission']);
    mockMissionSessionService = jasmine.createSpyObj('MissionSessionService', [
      'rehydrateActiveMission',
      'markMissionStarted',
      'completeMission',
    ]);
    mockMissionDatabaseService = jasmine.createSpyObj('MissionDatabaseService', ['createMission']);

    await TestBed.configureTestingModule({
      imports: [SimulationPageComponent],
      providers: [
        { provide: Router, useValue: mockRouter },
        { provide: MissionService, useValue: mockMissionService },
        { provide: MissionSessionService, useValue: mockMissionSessionService },
        { provide: MissionDatabaseService, useValue: mockMissionDatabaseService },
      ],
    }).compileComponents();

    fixture = TestBed.createComponent(SimulationPageComponent);
    component = fixture.componentInstance;
    fixture.detectChanges();
  });

  it('devrait créer le composant', () => {
    expect(component).toBeTruthy();
  });

  it('devrait appeler rehydrateActiveMission lors du ngOnInit', () => {
    component.ngOnInit();
    expect(mockMissionSessionService.rehydrateActiveMission).toHaveBeenCalled();
  });

  it('devrait démarrer une mission avec succès', () => {
    mockMissionService.startMission.and.returnValue(of({ message: 'started' }));
    component.startMission();
    expect(component.message).toBe('Mission demandée.');
    expect(mockMissionSessionService.markMissionStarted).toHaveBeenCalled();
    expect(mockMissionService.startMission).toHaveBeenCalled();
  });

  it('devrait gérer une erreur lors du démarrage de mission', () => {
    const error = new HttpErrorResponse({ error: 'fail', status: 500 });
    mockMissionService.startMission.and.returnValue(throwError(() => error));
    spyOn(console, 'error');
    component.startMission();
    expect(console.error).toHaveBeenCalledWith('Error starting mission:', error);
  });

  it('devrait arrêter une mission avec succès', fakeAsync(() => {
    mockMissionService.cancelMission.and.returnValue(of({}));
    spyOn<any>(component, 'finalizeMission');
    component.stopMission();
    tick();
    expect(component.message).toBe('Mission terminée.');
    expect(mockMissionService.cancelMission).toHaveBeenCalled();
    expect(component['finalizeMission']).toHaveBeenCalled();
  }));

  it('devrait gérer une erreur lors de l’arrêt de mission', fakeAsync(() => {
    const error = new HttpErrorResponse({ error: 'stop failed', status: 500 });
    mockMissionService.cancelMission.and.returnValue(throwError(() => error));
    spyOn<any>(component, 'finalizeMission');
    spyOn(console, 'error');
    component.stopMission();
    tick();
    expect(console.error).toHaveBeenCalledWith('Error stopping mission:', error);
    expect(component['finalizeMission']).toHaveBeenCalled();
  }));

  describe('finalizeMission', () => {
    it('devrait naviguer vers /home si aucune mission', fakeAsync(() => {
      mockMissionSessionService.completeMission.and.returnValue(Promise.resolve(null));
      component['finalizeMission']();
      tick();
      expect(mockRouter.navigate).toHaveBeenCalledWith(['/home']);
    }));

    it('devrait gérer une erreur dans completeMission', fakeAsync(() => {
      mockMissionSessionService.completeMission.and.returnValue(Promise.reject('error'));
      spyOn(console, 'error');
      component['finalizeMission']();
      tick();
      expect(console.error).toHaveBeenCalledWith('Erreur lors de la finalisation de la mission:', 'error');
      expect(mockRouter.navigate).toHaveBeenCalledWith(['/home']);
    }));
  });

  it('devrait changer le robot sélectionné', () => {
    component.setSelectedRobot('limo2');
    expect(component.selectedRobotId).toBe('limo2');
  });
});
