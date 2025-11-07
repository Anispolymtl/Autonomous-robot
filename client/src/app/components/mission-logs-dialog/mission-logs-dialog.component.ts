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
    private readonly timestampKeys = ['timestamp', 'time', 'date', 'createdAt'];
    private readonly levelKeys = ['level', 'severity', 'status'];
    private readonly phaseKeys = ['phase', 'state', 'stage'];
    private readonly primaryMessageKeys = ['message', 'event', 'action', 'description', 'text', 'log', 'entry', 'value', 'info', 'payload', 'data'];
    private readonly detailKeys = ['details', 'description', 'payload', 'data'];

    readonly hiddenKeys = Array.from(
        new Set([
            ...this.timestampKeys,
            ...this.levelKeys,
            ...this.phaseKeys,
            ...this.primaryMessageKeys,
            ...this.detailKeys,
        ])
    );

    constructor(
        private dialogRef: MatDialogRef<MissionLogsDialogComponent>,
        @Inject(MAT_DIALOG_DATA) public data: { mission: Mission; logs?: MissionLogEntry[] }
    ) {}

    close(): void {
        this.dialogRef.close();
    }

    getTimestampLabel(log: MissionLogEntry, index: number): string {
        let candidate = this.resolveLogField(log, this.timestampKeys) as Date | string | number | undefined;

        if (candidate === undefined && (typeof log === 'string' || typeof log === 'number')) {
            candidate = log;
        }

        const formatted = this.formatTimestamp(candidate);
        return formatted ?? `Entrée #${index + 1}`;
    }

    getPhaseLabel(log: MissionLogEntry): string | null {
        const value = this.resolveLogField(log, this.phaseKeys);
        return typeof value === 'string' && value.trim() ? value : null;
    }

    getLevelLabel(log: MissionLogEntry): string | null {
        const raw = this.resolveLogField(log, this.levelKeys);
        const label = typeof raw === 'string' && raw.trim() ? raw : null;
        return label ? label.toUpperCase() : null;
    }

    getLevelClass(label: string | null): string {
        if (!label) return '';
        const normalized = label.toLowerCase();
        if (normalized.includes('error') || normalized.includes('fail') || normalized.includes('critical')) return 'error';
        if (normalized.includes('warn') || normalized.includes('alert')) return 'warn';
        return 'info';
    }

    getPrimaryMessage(log: MissionLogEntry, index: number): string {
        const candidate = this.resolveLogField(log, this.primaryMessageKeys);

        if (candidate !== undefined && candidate !== null && candidate !== '') {
            return typeof candidate === 'string' ? candidate : this.formatValue(candidate);
        }

        if (typeof log === 'string' || typeof log === 'number' || typeof log === 'boolean') {
            return String(log);
        }

        return `Entrée #${index + 1}`;
    }

    getDetails(log: MissionLogEntry): string | null {
        const candidate = this.resolveLogField(log, this.detailKeys);
        if (candidate === null || candidate === undefined || candidate === '') {
            return null;
        }
        return typeof candidate === 'string' ? candidate : this.formatValue(candidate);
    }

    getAttributeEntries(log: MissionLogEntry): { key: string; value: unknown }[] | null {
        if (!this.isLogObject(log)) {
            return null;
        }

        const entries = Object.entries(log).filter(
            ([key, value]) => !this.hiddenKeys.includes(key) && value !== undefined && value !== null && value !== ''
        );

        return entries.length > 0 ? entries.map(([key, value]) => ({ key, value })) : null;
    }

    formatValue(value: unknown): string {
        if (value === null || value === undefined || value === '') {
            return '—';
        }
        if (value instanceof Date) {
            return value.toLocaleString('fr-CA');
        }
        if (typeof value === 'object') {
            return JSON.stringify(value, null, 2);
        }
        return String(value);
    }

    private formatTimestamp(value?: Date | string | number): string | null {
        if (value === null || value === undefined) {
            return null;
        }

        if (value instanceof Date) {
            return value.toLocaleString('fr-CA');
        }

        const numeric = typeof value === 'number' ? value : Number(value);
        if (!Number.isNaN(numeric)) {
            const dateFromNumber = new Date(numeric > 1e11 ? numeric : numeric * 1000);
            if (!Number.isNaN(dateFromNumber.getTime())) {
                return dateFromNumber.toLocaleString('fr-CA');
            }
        }

        const parsed = new Date(value);
        if (!Number.isNaN(parsed.getTime())) {
            return parsed.toLocaleString('fr-CA');
        }

        return String(value);
    }

    private resolveLogField(log: MissionLogEntry, keys: string[]): unknown {
        if (!this.isLogObject(log)) {
            return undefined;
        }

        for (const key of keys) {
            const value = log[key];
            if (value !== undefined && value !== null && value !== '') {
                return value;
            }
        }

        return undefined;
    }

    private isLogObject(log: MissionLogEntry): log is Record<string, unknown> {
        return typeof log === 'object' && log !== null && !Array.isArray(log);
    }
}
