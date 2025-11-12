import { ComponentFixture, TestBed } from '@angular/core/testing';
import { of, throwError } from 'rxjs';
import { MissionListComponent } from './mission-list.component';
import { MissionDatabaseService } from '@app/services/mission-database.service';
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
        missionServiceSpy.getMissionsByMode.and.returnValue(of([]));
        component.filterByMode('REAL');

        expect(component.selectedMode).toBe('REAL');
        expect(missionServiceSpy.getMissionsByMode).toHaveBeenCalledWith('REAL');
    });

    it('should clear filters and load all missions', () => {
        missionServiceSpy.getAllMissions.and.returnValue(of([]));
        component.selectedMode = 'SIMULATION';
        component.clearFilters();

        expect(component.selectedMode).toBeNull();
        expect(missionServiceSpy.getAllMissions).toHaveBeenCalled();
    });

    it('should reload data even if delete mission returns error', () => {
        spyOn(window, 'confirm').and.returnValue(true);
        const error = new HttpErrorResponse({ status: 400 });
        missionServiceSpy.deleteMission.and.returnValue(throwError(() => error));
        missionServiceSpy.getAllMissions.and.returnValue(of([]));
        missionServiceSpy.getMissionStats.and.returnValue(of({} as MissionStats));

        component.deleteMission('456');

        expect(missionServiceSpy.deleteMission).toHaveBeenCalled();
        expect(missionServiceSpy.getAllMissions).toHaveBeenCalled();
        expect(missionServiceSpy.getMissionStats).toHaveBeenCalled();
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
