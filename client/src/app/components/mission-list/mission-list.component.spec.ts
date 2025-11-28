import { ComponentFixture, TestBed } from '@angular/core/testing';
import { of, throwError } from 'rxjs';
import { MissionListComponent } from './mission-list.component';
import { MissionDatabaseService } from '@app/services/mission-database/mission-database.service';
import { MatDialog } from '@angular/material/dialog';
import { HttpErrorResponse } from '@angular/common/http';
import { Mission, MissionLogEntry, MissionStats } from '@app/interfaces/mission';

describe('MissionListComponent', () => {
    let component: MissionListComponent;
    let fixture: ComponentFixture<MissionListComponent>;
    let missionServiceSpy: jasmine.SpyObj<MissionDatabaseService>;
    let dialogSpy: jasmine.SpyObj<MatDialog>;

    beforeEach(async () => {
        missionServiceSpy = jasmine.createSpyObj('MissionDatabaseService', [
            'getAllMissions',
            'getMissionsByMode',
            'getMissionById',
            'getMissionStats',
            'deleteMission'
        ]);
        dialogSpy = jasmine.createSpyObj('MatDialog', ['open']);

        await TestBed.configureTestingModule({
            imports: [MissionListComponent],
            providers: [
                { provide: MissionDatabaseService, useValue: missionServiceSpy },
                { provide: MatDialog, useValue: dialogSpy }
            ]
        }).compileComponents();

        fixture = TestBed.createComponent(MissionListComponent);
        component = fixture.componentInstance;
        missionServiceSpy.getAllMissions.and.returnValue(of([]));
        missionServiceSpy.getMissionsByMode.and.returnValue(of([]));
        missionServiceSpy.getMissionStats.and.returnValue(of({} as MissionStats));
    });

    it('should create', () => {
        expect(component).toBeTruthy();
    });

    it('should handle error when loading missions', () => {
        const error = new HttpErrorResponse({ status: 500, statusText: 'Server Error' });
        missionServiceSpy.getAllMissions.and.returnValue(throwError(() => error));

        component.loadMissions();

        expect(component.error).toContain('Erreur lors du chargement des missions');
        expect(component.loading).toBeFalse();
    });

    it('should filter missions by mode', () => {
        component.filterByMode('REAL');

        expect(component.selectedMode).toBe('REAL');
        expect(missionServiceSpy.getMissionsByMode).toHaveBeenCalledWith('REAL', component.pageSize, 0);
    });

    it('should clear filters and load all missions', () => {
        component.selectedMode = 'SIMULATION';
        component.clearFilters();

        expect(component.selectedMode).toBeNull();
        expect(missionServiceSpy.getAllMissions).toHaveBeenCalledWith(component.pageSize, 0);
    });

    it('should reload data even if delete mission returns error', () => {
        spyOn(window, 'confirm').and.returnValue(true);
        const error = new HttpErrorResponse({ status: 400 });
        missionServiceSpy.deleteMission.and.returnValue(throwError(() => error));
        missionServiceSpy.getAllMissions.and.returnValue(of([]));
        missionServiceSpy.getMissionStats.and.returnValue(of({} as MissionStats));

        component.deleteMission('456');

        expect(missionServiceSpy.deleteMission).toHaveBeenCalled();
        expect(missionServiceSpy.getAllMissions).toHaveBeenCalledWith(component.pageSize, 0);
        expect(missionServiceSpy.getMissionStats).toHaveBeenCalled();
    });

    it('should load more missions with pagination', () => {
        const baseMission = {
            missionName: 'M',
            robots: [],
            mode: 'SIMULATION',
            distance: 0,
            durationSec: 0
        } as Mission;
        const firstBatch = Array(component.pageSize).fill(baseMission);
        const secondBatch = [{ ...baseMission, missionName: 'M7' }];

        missionServiceSpy.getAllMissions.and.returnValues(of(firstBatch), of(secondBatch));

        component.loadMissions(true);
        component.loadMore();

        expect(missionServiceSpy.getAllMissions.calls.argsFor(0)).toEqual([component.pageSize, 0]);
        expect(missionServiceSpy.getAllMissions.calls.argsFor(1)).toEqual([component.pageSize, firstBatch.length]);
        expect(component.missions.length).toBe(firstBatch.length + secondBatch.length);
        expect(component.allLoaded).toBeTrue();
    });

    it('should mark all missions loaded when total count is reached', () => {
        const missions = Array(component.pageSize).fill({
            missionName: 'M',
            robots: [],
            mode: 'SIMULATION',
            distance: 0,
            durationSec: 0
        } as Mission);
        const stats: MissionStats = {
            total: component.pageSize,
            byRobot: {},
            byMode: {},
            totalDistance: 0,
            averageDuration: 0
        };

        missionServiceSpy.getAllMissions.and.returnValue(of(missions));
        missionServiceSpy.getMissionStats.and.returnValue(of(stats));

        component.loadMissions(true);
        component.loadStats();

        expect(component.missions.length).toBe(component.pageSize);
        expect(component.allLoaded).toBeTrue();
    });

    it('should not delete mission if user cancels', () => {
        spyOn(window, 'confirm').and.returnValue(false);
        component.deleteMission('789');
        expect(missionServiceSpy.deleteMission).not.toHaveBeenCalled();
    });

    it('should format date correctly', () => {
        const date = new Date('2023-05-20T10:00:00Z');
        const result = component.formatDate(date);
        expect(result).toContain('2023');
    });

    it('should format duration correctly', () => {
        expect(component.formatDuration(65)).toBe('1m 5s');
        expect(component.formatDuration(3605)).toBe('1h 0m 5s');
        expect(component.formatDuration(5)).toBe('5s');
    });

    it('should format distance correctly', () => {
        expect(component.formatDistance(500)).toBe('500 m');
        expect(component.formatDistance(1500)).toBe('1.50 km');
    });

    it('should format robots correctly', () => {
        expect(component.formatRobots(['R1', 'R2'])).toBe('R1 & R2');
        expect(component.formatRobots([])).toBe('N/A');
        expect(component.formatRobots(undefined)).toBe('N/A');
    });

    it('should extract logs from mission with various keys', () => {
        const logEntries = [{ message: 'ok' }] as unknown as MissionLogEntry[];
        const mission = { logEntries } as unknown as Mission;
        const result = (component as any).extractLogsFromMission(mission);
        expect(result).toEqual(logEntries);
    });

    it('should return empty array when no logs found', () => {
        const mission = {} as Mission;
        const result = (component as any).extractLogsFromMission(mission);
        expect(result).toEqual([]);
    });
});
