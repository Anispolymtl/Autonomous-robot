import { ComponentFixture, TestBed, fakeAsync, tick } from '@angular/core/testing';
import { RobotLoginComponent } from './robot-login.component';
import { Router } from '@angular/router';
import { MissionSessionService } from '@app/services/mission-session/mission-session.service';
import { CommonModule } from '@angular/common';
import { FormsModule } from '@angular/forms';

interface MissionCreatedPayload {
  missionId: string;
  mission: any;
  missionName: string;
  mode: 'REAL' | 'SIMULATION';
}

describe('RobotLoginComponent', () => {
  let component: RobotLoginComponent;
  let fixture: ComponentFixture<RobotLoginComponent>;
  let routerSpy: jasmine.SpyObj<Router>;
  let missionSessionSpy: jasmine.SpyObj<MissionSessionService>;

  const mockMissionPayload: MissionCreatedPayload = {
    missionId: '123',
    mission: {},
    missionName: 'Test Mission',
    mode: 'REAL'
  };

  beforeEach(async () => {
    routerSpy = jasmine.createSpyObj('Router', ['navigate']);
    missionSessionSpy = jasmine.createSpyObj('MissionSessionService', ['initializeMission']);

    await TestBed.configureTestingModule({
      imports: [RobotLoginComponent],
    })
    .overrideComponent(RobotLoginComponent, {
      set: {
        imports: [CommonModule, FormsModule]
      }
    })
    .overrideProvider(Router, { useValue: routerSpy })
    .overrideProvider(MissionSessionService, { useValue: missionSessionSpy })
    .compileComponents();
  });

  beforeEach(() => {
    fixture = TestBed.createComponent(RobotLoginComponent);
    component = fixture.componentInstance;
    fixture.detectChanges();
  });

  it('should create', () => {
    expect(component).toBeTruthy();
  });

  describe('selectMode', () => {
    it('should set the selectedMode', () => {
      component.selectMode('real');
      expect(component.selectedMode).toBe('real');
      component.selectMode('simulation');
      expect(component.selectedMode).toBe('simulation');
    });
  });

  describe('isMissionNameValid', () => {
    it('should return false if missionName is empty', () => {
      component.missionName = '';
      expect(component.isMissionNameValid).toBeFalse();
    });

    it('should return true if missionName has text', () => {
      component.missionName = 'Test Mission';
      expect(component.isMissionNameValid).toBeTrue();
    });
  });

  describe('onMissionNameBlur', () => {
    it('should mark missionNameTouched as true', () => {
      expect(component.missionNameTouched).toBeFalse();
      component.onMissionNameBlur();
      expect(component.missionNameTouched).toBeTrue();
    });
  });

  describe('onModeSubmit', () => {
    it('should do nothing if mode or missionName is invalid', fakeAsync(async () => {
      component.selectedMode = null;
      component.missionName = '';
      await component.onModeSubmit();
      expect(routerSpy.navigate).not.toHaveBeenCalled();

      component.selectedMode = 'real';
      component.missionName = '';
      await component.onModeSubmit();
      expect(routerSpy.navigate).not.toHaveBeenCalled();

      component.selectedMode = null;
      component.missionName = 'Test';
      await component.onModeSubmit();
      expect(routerSpy.navigate).not.toHaveBeenCalled();
    }));

    it('should call initializeMission and navigate to /real-mode', fakeAsync(async () => {
      component.selectedMode = 'real';
      component.missionName = 'Mission 1';
      missionSessionSpy.initializeMission.and.returnValue(Promise.resolve(mockMissionPayload));

      await component.onModeSubmit();
      tick();

      expect(missionSessionSpy.initializeMission).toHaveBeenCalledWith('Mission 1', 'REAL');
      expect(routerSpy.navigate).toHaveBeenCalledWith(['/real-mode']);
      expect(component.error).toBeNull();
    }));

    it('should call initializeMission and navigate to /simulation-mode', fakeAsync(async () => {
      component.selectedMode = 'simulation';
      component.missionName = 'Sim Mission';
      missionSessionSpy.initializeMission.and.returnValue(Promise.resolve({
        ...mockMissionPayload,
        mode: 'SIMULATION' as const
      }));

      await component.onModeSubmit();
      tick();

      expect(missionSessionSpy.initializeMission).toHaveBeenCalledWith('Sim Mission', 'SIMULATION');
      expect(routerSpy.navigate).toHaveBeenCalledWith(['/simulation-mode']);
      expect(component.error).toBeNull();
    }));

    it('should set error message if initializeMission rejects', fakeAsync(async () => {
      component.selectedMode = 'real';
      component.missionName = 'Mission Fail';
      missionSessionSpy.initializeMission.and.returnValue(Promise.reject(new Error('Fail init')));

      await component.onModeSubmit();
      tick();

      expect(component.error).toBe('Fail init');
      expect(routerSpy.navigate).not.toHaveBeenCalled();
    }));

    it('should trim missionName before submitting', fakeAsync(async () => {
      component.selectedMode = 'simulation';
      component.missionName = '   TrimTest   ';
      missionSessionSpy.initializeMission.and.returnValue(Promise.resolve({
        ...mockMissionPayload,
        mode: 'SIMULATION' as const
      }));

      await component.onModeSubmit();
      tick();

      expect(missionSessionSpy.initializeMission).toHaveBeenCalledWith('TrimTest', 'SIMULATION');
    }));
  });
});
