import { ComponentFixture, TestBed, fakeAsync, tick } from '@angular/core/testing';
import { LogsPageComponent } from './logs-page.component';
import { ActivatedRoute } from '@angular/router';
import { of } from 'rxjs';

describe('LogsPageComponent', () => {
  let component: LogsPageComponent;
  let fixture: ComponentFixture<LogsPageComponent>;
  let activatedRouteSpy: jasmine.SpyObj<ActivatedRoute>;

  beforeEach(async () => {
    activatedRouteSpy = jasmine.createSpyObj('ActivatedRoute', [], {
      queryParamMap: of(new Map([
        ['missionId', '123'],
        ['missionName', 'Test Mission']
      ]))
    });

    await TestBed.configureTestingModule({
      imports: [LogsPageComponent],
      providers: [
        { provide: ActivatedRoute, useValue: activatedRouteSpy }
      ]
    }).compileComponents();
  });

  beforeEach(() => {
    fixture = TestBed.createComponent(LogsPageComponent);
    component = fixture.componentInstance;
    fixture.detectChanges();
  });

  it('should create', () => {
    expect(component).toBeTruthy();
  });

  it('should initialize missionId and missionName from query params', fakeAsync(() => {
    tick();
    expect(component.missionId).toBe('123');
    expect(component.missionName).toBe('Test Mission');
  }));

  describe('hasActiveMission', () => {
    it('should return true if missionId is set', () => {
      component.missionId = '123';
      component.missionName = null;
      expect(component.hasActiveMission).toBeTrue();
    });

    it('should return true if missionName is set', () => {
      component.missionId = null;
      component.missionName = 'Test Mission';
      expect(component.hasActiveMission).toBeTrue();
    });

    it('should return false if neither missionId nor missionName is set', () => {
      component.missionId = null;
      component.missionName = null;
      expect(component.hasActiveMission).toBeFalse();
    });
  });

  describe('switchTab', () => {
    it('should set activeTab to "live"', () => {
      component.switchTab('live');
      expect(component.activeTab).toBe('live');
    });

    it('should set activeTab to "history"', () => {
      component.switchTab('history');
      expect(component.activeTab).toBe('history');
    });
  });

  describe('objectKeys', () => {
    it('should return the keys of an object', () => {
      const obj = { a: 1, b: 2 };
      expect(component.objectKeys(obj)).toEqual(['a', 'b']);
    });
  });

  it('should have sample liveData entries', () => {
    expect(component.liveData.length).toBe(3);
    expect(component.liveData[0].action).toBe('start_mission');
    expect(component.liveData[1].details.x).toBe(0.21);
  });
});
