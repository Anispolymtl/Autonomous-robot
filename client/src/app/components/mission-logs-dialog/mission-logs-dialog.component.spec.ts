import { ComponentFixture, TestBed } from '@angular/core/testing';
import { MissionLogsDialogComponent } from './mission-logs-dialog.component';
import { MAT_DIALOG_DATA, MatDialogRef } from '@angular/material/dialog';
import { Mission, MissionLogEntry } from '@app/interfaces/mission';

describe('MissionLogsDialogComponent', () => {
    let component: MissionLogsDialogComponent;
    let fixture: ComponentFixture<MissionLogsDialogComponent>;
    let dialogRefSpy: jasmine.SpyObj<MatDialogRef<MissionLogsDialogComponent>>;

    const mockMission: Mission = {
        _id: '123',
        createdAt: new Date('2025-01-01T10:00:00Z'),
        durationSec: 120,
        robots: ['limo1', 'limo2'],
        mode: 'SIMULATION',
        distance: 15.2,
        missionName: 'Mission Test',
        logs: [],
        status: 'COMPLETED',
    };

    const mockLogs: MissionLogEntry[] = [
        {
            timestamp: '2025-01-01T12:00:00Z',
            robot: 'limo1',
            category: 'Command',
            action: 'move_forward',
            details: { distance: 5 },
        },
        {
            timestamp: 'invalid-date',
            robot: 'limo2',
            action: 'status_check',
            details: {},
            category: 'Command',
        },
    ];

    beforeEach(async () => {
        dialogRefSpy = jasmine.createSpyObj('MatDialogRef', ['close']);

        await TestBed.configureTestingModule({
            imports: [MissionLogsDialogComponent],
            providers: [
                { provide: MatDialogRef, useValue: dialogRefSpy },
                { provide: MAT_DIALOG_DATA, useValue: { mission: mockMission, logs: mockLogs } },
            ],
        }).compileComponents();

        fixture = TestBed.createComponent(MissionLogsDialogComponent);
        component = fixture.componentInstance;
        fixture.detectChanges();
    });

    it('should create the component', () => {
        expect(component).toBeTruthy();
    });

    it('should close the dialog when close() is called', () => {
        component.close();
        expect(dialogRefSpy.close).toHaveBeenCalled();
    });

    describe('formatTimestamp()', () => {
        it('should return the original value if the date is invalid', () => {
            const result = component.formatTimestamp('not-a-date');
            expect(result).toBe('not-a-date');
        });
    });

    describe('getDetailEntries()', () => {
        it('should return key-value pairs from the details field', () => {
            const log: MissionLogEntry = {
                timestamp: '2025-01-01T12:00:00Z',
                robot: 'limo1',
                category: 'Command',
                action: 'rotate',
                details: { angle: 90, speed: 0.5 },
            };
            const entries = component.getDetailEntries(log);
            expect(entries).toEqual([
                { key: 'angle', value: 90 },
                { key: 'speed', value: 0.5 },
            ]);
        });

        it('should return an empty array if details is empty', () => {
            const log: MissionLogEntry = {
                timestamp: '2025-01-01T12:00:00Z',
                robot: 'limo2',
                action: 'idle',
                details: {},
                category: 'Command',
            };
            const entries = component.getDetailEntries(log);
            expect(entries).toEqual([]);
        });
    });

    describe('formatValue()', () => {
        it('should return "—" for null, undefined, or empty string', () => {
            expect(component.formatValue(null)).toBe('—');
            expect(component.formatValue(undefined)).toBe('—');
            expect(component.formatValue('')).toBe('—');
        });

        it('should format a Date with the correct format', () => {
            const date = new Date('2025-01-01T12:00:00Z');
            const result = component.formatValue(date);
            expect(result).toContain('2025');
        });

        it('should convert an object to JSON string', () => {
            const obj = { a: 1, b: 'test' };
            const result = component.formatValue(obj);
            expect(result).toBe(JSON.stringify(obj));
        });

        it('should convert a number or string to text', () => {
            expect(component.formatValue(42)).toBe('42');
            expect(component.formatValue('abc')).toBe('abc');
        });
    });
});
