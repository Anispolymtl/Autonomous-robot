// import { ComponentFixture, TestBed } from '@angular/core/testing';
// import { of } from 'rxjs';
// import { MissionListComponent } from './mission-list.component';
// import { MissionDatabaseService } from '@app/services/mission-database/mission-database.service';
// import { MatDialog } from '@angular/material/dialog';
// import { Mission, MissionStats, MissionLogEntry } from '@app/interfaces/mission';

// describe('MissionListComponent', () => {
//   let component: MissionListComponent;
//   let fixture: ComponentFixture<MissionListComponent>;
//   let missionDbSpy: jasmine.SpyObj<MissionDatabaseService>;
//   let dialogSpy: jasmine.SpyObj<MatDialog>;

//   const mockMissions: Mission[] = [
//     {
//       id: '1',
//       robots: ['R1'],
//       mode: 'SIMULATION',
//       missionName: 'Test Mission',
//       durationSec: 120,
//       distance: 500,
//       logs: [{ timestamp: 0, message: 'Start' }] as MissionLogEntry[],
//     }
//   ];

//   const mockStats: MissionStats = {
//     total: 1,
//     byRobot: { R1: 1 },
//     byMode: { SIMULATION: 1 },
//     totalDistance: 500,
//     averageDuration: 120
//   };

//   beforeEach(async () => {
//     const missionDb = jasmine.createSpyObj('MissionDatabaseService', [
//       'getAllMissions',
//       'getMissionsByRobot',
//       'getMissionsByMode',
//       'getMissionStats',
//       'deleteMission',
//       'populateDatabase'
//     ]);
//     const dialog = jasmine.createSpyObj('MatDialog', ['open']);

//     await TestBed.configureTestingModule({
//       imports: [MissionListComponent],
//       providers: [
//         { provide: MissionDatabaseService, useValue: missionDb },
//         { provide: MatDialog, useValue: dialog }
//       ]
//     }).compileComponents();

//     fixture = TestBed.createComponent(MissionListComponent);
//     component = fixture.componentInstance;
//     missionDbSpy = TestBed.inject(MissionDatabaseService) as jasmine.SpyObj<MissionDatabaseService>;
//     dialogSpy = TestBed.inject(MatDialog) as jasmine.SpyObj<MatDialog>;
//   });

//   it('should create', () => {
//     expect(component).toBeTruthy();
//   });

//   it('should load missions on init', () => {
//     missionDbSpy.getAllMissions.and.returnValue(of(mockMissions));
//     missionDbSpy.getMissionStats.and.returnValue(of(mockStats));

//     component.ngOnInit();

//     expect(component.missions).toEqual(mockMissions);
//     expect(component.stats).toEqual(mockStats);
//   });

//   it('should filter by robot', () => {
//     missionDbSpy.getMissionsByRobot.and.returnValue(of([mockMissions[0]]));
//     component.filterByRobot('R1');
//     expect(component.selectedRobot).toBe('R1');
//     expect(component.selectedMode).toBeNull();
//     expect(component.missions).toEqual([mockMissions[0]]);
//   });

//   it('should filter by mode', () => {
//     missionDbSpy.getMissionsByMode.and.returnValue(of([mockMissions[0]]));
//     component.filterByMode('SIMULATION');
//     expect(component.selectedMode).toBe('SIMULATION');
//     expect(component.selectedRobot).toBeNull();
//     expect(component.missions).toEqual([mockMissions[0]]);
//   });

//   it('should clear filters', () => {
//     missionDbSpy.getAllMissions.and.returnValue(of(mockMissions));
//     component.clearFilters();
//     expect(component.selectedRobot).toBeNull();
//     expect(component.selectedMode).toBeNull();
//     expect(component.missions).toEqual(mockMissions);
//   });

//   it('should extract logs from mission', () => {
//     const logs = component['extractLogsFromMission'](mockMissions[0]);
//     expect(logs!).toEqual(mockMissions[0].logs!);
//   });

//   it('should format date', () => {
//     const date = new Date('2025-11-10T12:00:00Z');
//     expect(component.formatDate(date)).toBe(date.toLocaleString('fr-FR'));
//     expect(component.formatDate(undefined)).toBe('N/A');
//   });

//   it('should format duration', () => {
//     expect(component.formatDuration(3665)).toBe('1h 1m 5s');
//     expect(component.formatDuration(65)).toBe('1m 5s');
//     expect(component.formatDuration(5)).toBe('5s');
//   });

//   it('should format distance', () => {
//     expect(component.formatDistance(500)).toBe('500 m');
//     expect(component.formatDistance(1500)).toBe('1.50 km');
//   });

//   it('should format robots', () => {
//     expect(component.formatRobots(['R1', 'R2'])).toBe('R1 & R2');
//     expect(component.formatRobots([])).toBe('N/A');
//     expect(component.formatRobots(undefined)).toBe('N/A');
//   });

//   it('should get unique robots', () => {
//     component.missions = [
//       { ...mockMissions[0], robots: ['R1', 'R2'] },
//       { ...mockMissions[0], robots: ['R2', 'R3'] }
//     ];
//     expect(component.getUniqueRobots()).toEqual(['R1', 'R2', 'R3']);
//   });

//   it('should open logs dialog', () => {
//     component.openLogsDialog(mockMissions[0]);
//     expect(dialogSpy.open).toHaveBeenCalled();
//   });
// });
