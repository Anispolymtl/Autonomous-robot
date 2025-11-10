// import { ComponentFixture, TestBed } from '@angular/core/testing';
// import { MissionLogsDialogComponent } from './mission-logs-dialog.component';
// import { MAT_DIALOG_DATA, MatDialogRef } from '@angular/material/dialog';
// import { Mission, MissionLogEntry } from '@app/interfaces/mission';

// describe('MissionLogsDialogComponent - full coverage', () => {
//   let component: MissionLogsDialogComponent;
//   let fixture: ComponentFixture<MissionLogsDialogComponent>;
//   let dialogRefSpy: jasmine.SpyObj<MatDialogRef<MissionLogsDialogComponent>>;

//   const complexLogObject = {
//     timestamp: 1672531200000,
//     level: 'error',
//     phase: 'start',
//     message: 'Complex log',
//     details: 'Some details',
//     extraField: 'Extra',
//     emptyField: '',
//     nullField: null,
//     undefinedField: undefined
//   };

//   const allLogTypes: MissionLogEntry[] = [
//     'Simple string log',
//     12345,
//     true,
//     false,
//     null,
//     undefined,
//     complexLogObject,
//     { message: '', details: '' }, // empty object values
//     {}, // completely empty object
//     { timestamp: 'invalid date', level: 'warn' } // invalid timestamp string
//   ];

//   const mockMission: Mission = {
//     _id: '1',
//     robots: ['R1', 'R2'],
//     mode: 'SIMULATION',
//     missionName: 'Test Mission',
//     durationSec: 3600,
//     distance: 1200,
//     logs: allLogTypes
//   };

//   beforeEach(async () => {
//     dialogRefSpy = jasmine.createSpyObj('MatDialogRef', ['close']);

//     await TestBed.configureTestingModule({
//       imports: [MissionLogsDialogComponent],
//       providers: [
//         { provide: MatDialogRef, useValue: dialogRefSpy },
//         { provide: MAT_DIALOG_DATA, useValue: { mission: mockMission, logs: mockMission.logs } }
//       ]
//     }).compileComponents();

//     fixture = TestBed.createComponent(MissionLogsDialogComponent);
//     component = fixture.componentInstance;
//     fixture.detectChanges();
//   });

//   it('should create', () => {
//     expect(component).toBeTruthy();
//   });

//   it('should close the dialog', () => {
//     component.close();
//     expect(dialogRefSpy.close).toHaveBeenCalled();
//   });

//   it('should get timestamp labels correctly', () => {
//     expect(component.getTimestampLabel(allLogTypes[0], 0)).toBe('Entrée #1'); // string
//     expect(component.getTimestampLabel(allLogTypes[1], 1)).toContain('1970'); // number
//     expect(component.getTimestampLabel(allLogTypes[2], 2)).toBe('Entrée #3'); // boolean true
//     expect(component.getTimestampLabel(allLogTypes[3], 3)).toBe('Entrée #4'); // boolean false
//     expect(component.getTimestampLabel(allLogTypes[4], 4)).toBe('Entrée #5'); // null
//     expect(component.getTimestampLabel(allLogTypes[5], 5)).toBe('Entrée #6'); // undefined
//     expect(component.getTimestampLabel(allLogTypes[6], 6)).toContain('2023'); // complex object timestamp
//     expect(component.getTimestampLabel(allLogTypes[9], 9)).toBe('Entrée #10'); // invalid date string
//   });

//   it('should get phase labels', () => {
//     expect(component.getPhaseLabel(allLogTypes[6])).toBe('start'); // complex object
//     expect(component.getPhaseLabel(allLogTypes[0])).toBeNull(); // string
//     expect(component.getPhaseLabel(allLogTypes[7])).toBeNull(); // object with empty strings
//   });

//   it('should get level labels and classes', () => {
//     expect(component.getLevelLabel(allLogTypes[6])).toBe('ERROR');
//     expect(component.getLevelClass('ERROR')).toBe('error');
//     expect(component.getLevelClass('WARN')).toBe('warn');
//     expect(component.getLevelClass('INFO')).toBe('info');
//     expect(component.getLevelClass(null)).toBe('');
//   });

//   it('should get primary messages', () => {
//     expect(component.getPrimaryMessage(allLogTypes[0], 0)).toBe('Simple string log');
//     expect(component.getPrimaryMessage(allLogTypes[1], 1)).toBe('12345');
//     expect(component.getPrimaryMessage(allLogTypes[2], 2)).toBe('true');
//     expect(component.getPrimaryMessage(allLogTypes[5], 5)).toBe('Entrée #6'); // undefined
//     expect(component.getPrimaryMessage(allLogTypes[6], 6)).toBe('Complex log');
//     expect(component.getPrimaryMessage(allLogTypes[7], 7)).toBe('Entrée #8'); // empty values
//     expect(component.getPrimaryMessage(allLogTypes[8], 8)).toBe('Entrée #9'); // empty object
//   });

//   it('should get details', () => {
//     expect(component.getDetails(allLogTypes[6])).toBe('Some details'); // complex object
//     expect(component.getDetails(allLogTypes[0])).toBeNull(); // string
//     expect(component.getDetails(allLogTypes[7])).toBeNull(); // empty object values
//     expect(component.getDetails(allLogTypes[8])).toBeNull(); // empty object
//   });

//   it('should get attribute entries', () => {
//     const entries = component.getAttributeEntries(allLogTypes[6]);
//     expect(entries).toEqual([{ key: 'extraField', value: 'Extra' }]); // only non-hidden, non-empty fields
//     expect(component.getAttributeEntries(allLogTypes[0])).toBeNull(); // string
//     expect(component.getAttributeEntries(allLogTypes[8])).toBeNull(); // empty object
//   });

//   it('should format values correctly', () => {
//     expect(component.formatValue(null)).toBe('—');
//     expect(component.formatValue(undefined)).toBe('—');
//     expect(component.formatValue('')).toBe('—');
//     expect(component.formatValue(123)).toBe('123');
//     expect(component.formatValue(true)).toBe('true');
//     const date = new Date('2025-01-01T12:00:00');
//     expect(component.formatValue(date)).toBe(date.toLocaleString('fr-CA'));
//     const obj = { a: 1 };
//     expect(component.formatValue(obj)).toBe(JSON.stringify(obj, null, 2));
//   });
// });
