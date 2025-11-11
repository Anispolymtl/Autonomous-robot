import { ComponentFixture, TestBed } from '@angular/core/testing';
import { MissionListComponent } from './mission-list.component';
import { MissionDatabaseService } from '@app/services/mission-database.service';
import { MatDialog } from '@angular/material/dialog';
import { of } from 'rxjs';
import { Mission } from '@app/interfaces/mission';
import { MissionStats } from '@app/interfaces/mission';

describe('MissionListComponent', () => {
  let component: MissionListComponent;
  let fixture: ComponentFixture<MissionListComponent>;
  let missionDatabaseServiceSpy: jasmine.SpyObj<MissionDatabaseService>;
  let matDialogSpy: jasmine.SpyObj<MatDialog>;

  const mockMissions: Mission[] = [
    {
      durationSec: 120,
      robots: ['robot1'],
      mode: 'REAL',
      distance: 100,
      missionName: 'Mission 1',
    },
    {
      durationSec: 60,
      robots: ['robot2'],
      mode: 'SIMULATION',
      distance: 50,
      missionName: 'Mission 2',
    },
  ];

  const mockStats: MissionStats = {
    total: 2,
    byRobot: { robot1: 1, robot2: 1 },
    byMode: { REAL: 1, SIMULATION: 1 },
    totalDistance: 150,
    averageDuration: 90,
  };

  beforeEach(async () => {
    missionDatabaseServiceSpy = jasmine.createSpyObj('MissionDatabaseService', [
      'getAllMissions',
      'getMissionsByRobot',
      'getMissionsByMode',
      'getMissionStats',
      'deleteMission',
      'populateDatabase',
    ]);

    matDialogSpy = jasmine.createSpyObj('MatDialog', ['open']);

    await TestBed.configureTestingModule({
      imports: [MissionListComponent],
      providers: [
        { provide: MissionDatabaseService, useValue: missionDatabaseServiceSpy },
        { provide: MatDialog, useValue: matDialogSpy },
      ],
    }).compileComponents();

    fixture = TestBed.createComponent(MissionListComponent);
    component = fixture.componentInstance;
  });

  it('should create', () => {
    missionDatabaseServiceSpy.getAllMissions.and.returnValue(of(mockMissions));
    missionDatabaseServiceSpy.getMissionStats.and.returnValue(of(mockStats));
    fixture.detectChanges();
    expect(component).toBeTruthy();
  });

  it('should load missions successfully', () => {
    missionDatabaseServiceSpy.getAllMissions.and.returnValue(of(mockMissions));
    component.loadMissions();
    expect(component.loading).toBeFalse();
    expect(component.missions.length).toBe(2);
  });

  it('should format distance correctly', () => {
    expect(component.formatDistance(999)).toBe('999 m');
    expect(component.formatDistance(1200)).toBe('1.20 km');
  });

  it('should format duration correctly', () => {
    expect(component.formatDuration(45)).toBe('45s');
    expect(component.formatDuration(125)).toBe('2m 5s');
    expect(component.formatDuration(3661)).toBe('1h 1m 1s');
  });

  it('should return unique robot names', () => {
    component.missions = mockMissions;
    expect(component.getUniqueRobots()).toEqual(['robot1', 'robot2']);
  });

  it('should format robots list correctly', () => {
    expect(component.formatRobots(['r1', 'r2'])).toBe('r1 & r2');
    expect(component.formatRobots([])).toBe('N/A');
  });
});
