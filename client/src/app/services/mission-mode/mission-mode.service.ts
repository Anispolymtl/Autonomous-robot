import { Injectable } from '@angular/core';
import { HttpClient } from '@angular/common/http';
import { BehaviorSubject, Observable, firstValueFrom, map, of, tap } from 'rxjs';
import { catchError } from 'rxjs/operators';
import { environment } from 'src/environments/environment';
import { Mission } from '@app/interfaces/mission';

export type MissionMode = 'REAL' | 'SIMULATION' | null;

interface MissionModeResponse {
    mode: MissionMode;
}

export interface ActiveMissionResponse extends Mission {
    missionId: string;
}

@Injectable({
    providedIn: 'root',
})
export class MissionModeService {
    private modeSubject = new BehaviorSubject<MissionMode | undefined>(undefined);
    readonly mode$ = this.modeSubject.asObservable();
    private missionApi = `${environment.serverUrl}/api/mission`;
    private pollingHandle: any = null;

    constructor(private http: HttpClient) {}

    get currentMode(): MissionMode | undefined {
        return this.modeSubject.value;
    }

    refreshMode(): Observable<MissionMode> {
        return this.http.get<MissionModeResponse>(`${this.missionApi}/current-mode`).pipe(
            map((res) => res.mode ?? null),
            tap((mode) => this.modeSubject.next(mode)),
        );
    }

    ensureModeLoaded(): Promise<MissionMode> {
        const current = this.modeSubject.value;
        if (current !== undefined) {
            return Promise.resolve(current ?? null);
        }
        return firstValueFrom(this.refreshMode().pipe(catchError(() => of(null))));
    }

    setMode(mode: MissionMode): void {
        this.modeSubject.next(mode);
    }

    fetchActiveMission(): Observable<ActiveMissionResponse | null> {
        return this.http.get<ActiveMissionResponse | null>(`${this.missionApi}/active`).pipe(
            catchError(() => of(null))
        );
    }

    startPolling(intervalMs = 5000): void {
        if (this.pollingHandle) return;
        this.pollingHandle = setInterval(() => {
            this.refreshMode()
                .pipe(catchError(() => of(null)))
                .subscribe();
        }, intervalMs);
    }

    stopPolling(): void {
        if (this.pollingHandle) {
            clearInterval(this.pollingHandle);
            this.pollingHandle = null;
        }
    }
}
