import { CommonModule } from '@angular/common';
import { Component, Inject } from '@angular/core';
import { MAT_DIALOG_DATA, MatDialogModule, MatDialogRef } from '@angular/material/dialog';
import { Mission } from '@app/interfaces/mission';

export type MissionLogLevel = 'INFO' | 'WARN' | 'ERROR';

export interface MissionLogEntry {
    timestamp: Date;
    level: MissionLogLevel;
    phase: string;
    message: string;
    details?: string;
}

@Component({
    selector: 'app-mission-logs-dialog',
    standalone: true,
    imports: [CommonModule, MatDialogModule],
    templateUrl: './mission-logs-dialog.component.html',
    styleUrls: ['./mission-logs-dialog.component.scss']
})
export class MissionLogsDialogComponent {
    constructor(
        private dialogRef: MatDialogRef<MissionLogsDialogComponent>,
        @Inject(MAT_DIALOG_DATA) public data: { mission: Mission; logs: MissionLogEntry[] }
    ) {}

    close(): void {
        this.dialogRef.close();
    }
}

