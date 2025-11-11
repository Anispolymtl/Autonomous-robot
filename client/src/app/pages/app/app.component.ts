import { Component, OnInit } from '@angular/core';
import { Router, RouterOutlet } from '@angular/router';
import { GlobalNavbarComponent } from '@app/components/global-navbar/global-navbar.component';
import { MissionModeService } from '@app/services/mission-mode.service';
import { MissionSessionService } from '@app/services/mission-session.service';
import { filter } from 'rxjs/operators';
import { TelemetryLoggingService } from '@app/services/telemetry-logging.service';

@Component({
    selector: 'app-root',
    templateUrl: './app.component.html',
    styleUrls: ['./app.component.scss'],
    imports: [RouterOutlet, GlobalNavbarComponent],
})
export class AppComponent implements OnInit {
    private syncing = false;
    private readonly autoRedirectOrigins = new Set(['/home', '/', '/robot-login']);

    constructor(
        private missionModeService: MissionModeService,
        private missionSessionService: MissionSessionService,
        private router: Router,
        private telemetryLoggingService: TelemetryLoggingService
    ) {}

    ngOnInit(): void {
        this.telemetryLoggingService.start();
        this.missionModeService.startPolling();
        this.missionModeService
            .ensureModeLoaded()
            .then((mode) => {
                if (mode) {
                    this.missionSessionService.rehydrateActiveMission();
                }
                this.redirectIfNeeded(mode);
            })
            .catch(() => {});
        this.missionModeService.mode$
            .pipe(filter((mode) => mode !== undefined))
            .subscribe((mode) => {
                if (mode) {
                    this.missionSessionService.rehydrateActiveMission();
                }
                this.redirectIfNeeded(mode ?? null);
            });
    }

    private redirectIfNeeded(mode: 'REAL' | 'SIMULATION' | null): void {
        if (this.syncing) return;
        this.syncing = true;

        const currentUrl = this.normalizeUrl(this.router.url);

        if (mode === 'REAL') {
            if (currentUrl === '/simulation-mode' || (currentUrl !== '/real-mode' && this.shouldForceMissionRoute(currentUrl))) {
                this.router.navigate(['/real-mode']).finally(() => (this.syncing = false));
                return;
            }
        }

        if (mode === 'SIMULATION') {
            if (currentUrl === '/real-mode' || (currentUrl !== '/simulation-mode' && this.shouldForceMissionRoute(currentUrl))) {
                this.router.navigate(['/simulation-mode']).finally(() => (this.syncing = false));
                return;
            }
        }

        if (mode === null && this.isMissionRoute(currentUrl)) {
            this.router.navigate(['/home']).finally(() => (this.syncing = false));
            return;
        }

        this.syncing = false;
    }

    private normalizeUrl(url: string): string {
        const [path] = url.split('?');
        if (!path || path === '/') {
            return '/';
        }
        return path.endsWith('/') && path.length > 1 ? path.slice(0, -1) : path;
    }

    private shouldForceMissionRoute(url: string): boolean {
        return this.autoRedirectOrigins.has(url);
    }

    private isMissionRoute(url: string): boolean {
        return url === '/simulation-mode' || url === '/real-mode';
    }
}
