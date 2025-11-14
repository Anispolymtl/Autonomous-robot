import { CommonModule } from '@angular/common';
import { Component, Inject } from '@angular/core';
import { MAT_DIALOG_DATA, MatDialogRef } from '@angular/material/dialog';
import { Mission, MissionLogEntry } from '@app/interfaces/mission';

@Component({
    selector: 'app-mission-logs-dialog',
    standalone: true,
    imports: [CommonModule],
    templateUrl: './mission-logs-dialog.component.html',
    styleUrls: ['./mission-logs-dialog.component.scss']
})
export class MissionLogsDialogComponent {
    constructor(
        private dialogRef: MatDialogRef<MissionLogsDialogComponent>,
        @Inject(MAT_DIALOG_DATA) public data: { mission: Mission; logs?: MissionLogEntry[] }
    ) {}

    close(): void {
        this.dialogRef.close();
    }

    formatTimestamp(timestamp: string): string {
        const parsed = new Date(timestamp);
        return Number.isNaN(parsed.getTime()) ? timestamp : parsed.toLocaleString('fr-CA');
    }

    getDetailEntries(log: MissionLogEntry): { key: string; value: unknown }[] {
        return Object.entries(log.details ?? {}).map(([key, value]) => ({ key, value }));
    }

    formatValue(value: unknown): string {
        if (value === null || value === undefined || value === '') {
            return '—';
        }
        if (value instanceof Date) {
            return value.toLocaleString('fr-CA');
        }
        if (typeof value === 'object') {
            return JSON.stringify(value);
        }
        return String(value);
    }
}
