import { ComponentFixture, TestBed } from '@angular/core/testing';
import { MissionLogsDialogComponent } from './mission-logs-dialog.component';
import { MAT_DIALOG_DATA, MatDialogRef } from '@angular/material/dialog';
import { Mission, MissionLogEntry } from '@app/interfaces/mission';

describe('MissionLogsDialogComponent', () => {
  let component: MissionLogsDialogComponent;
  let fixture: ComponentFixture<MissionLogsDialogComponent>;
  let dialogRefSpy: jasmine.SpyObj<MatDialogRef<MissionLogsDialogComponent>>;

  const mockMission: Mission = {
    durationSec: 120,
    robots: ['robot1'],
    mode: 'REAL',
    distance: 100,
    missionName: 'Mission de test',
  };

  const mockLogs: MissionLogEntry[] = [
    { timestamp: new Date(), message: 'Mission démarrée', level: 'info' },
    { time: '2024-01-01T00:00:00Z', event: 'Erreur critique', level: 'error', details: 'Crash moteur' },
    'Entrée brute',
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

  it('should close the dialog', () => {
    component.close();
    expect(dialogRefSpy.close).toHaveBeenCalled();
  });

  it('should handle invalid timestamp gracefully', () => {
    const label = component.getTimestampLabel({} as MissionLogEntry, 1);
    expect(label).toContain('Entrée #');
  });

  it('should return phase label when present', () => {
    const log = { phase: 'INIT' } as MissionLogEntry;
    expect(component.getPhaseLabel(log)).toBe('INIT');
  });

  it('should return level label in uppercase', () => {
    const log = { level: 'warn' } as MissionLogEntry;
    expect(component.getLevelLabel(log)).toBe('WARN');
  });

  it('should return proper CSS class for level', () => {
    expect(component.getLevelClass('ERROR')).toBe('error');
    expect(component.getLevelClass('WARN')).toBe('warn');
    expect(component.getLevelClass('INFO')).toBe('info');
  });

  it('should return primary message correctly', () => {
    const log = { message: 'Test message' } as MissionLogEntry;
    expect(component.getPrimaryMessage(log, 0)).toBe('Test message');
  });

  it('should fallback to entry label if no message found', () => {
    const log = {} as MissionLogEntry;
    expect(component.getPrimaryMessage(log, 0)).toContain('Entrée #');
  });

  it('should format details when available', () => {
    const log = { details: 'Erreur fatale' } as MissionLogEntry;
    expect(component.getDetails(log)).toBe('Erreur fatale');
  });

  it('should return attribute entries excluding hidden keys', () => {
    const log = { custom: 42, message: 'hidden' } as MissionLogEntry;
    const entries = component.getAttributeEntries(log);
    expect(entries).toEqual([{ key: 'custom', value: 42 }]);
  });
});
