import { Component, OnInit } from '@angular/core';
import { Router, RouterOutlet } from '@angular/router';
import { GlobalNavbarComponent } from '@app/components/global-navbar/global-navbar.component';
import { MissionModeService } from '@app/services/mission-mode.service';
import { MissionSessionService } from '@app/services/mission-session.service';
import { filter } from 'rxjs/operators';

@Component({
    selector: 'app-root',
    templateUrl: './app.component.html',
    styleUrls: ['./app.component.scss'],
    imports: [RouterOutlet, GlobalNavbarComponent],
})
export class AppComponent implements OnInit {
    private syncing = false;

    constructor(
        private missionModeService: MissionModeService,
        private missionSessionService: MissionSessionService,
        private router: Router
    ) {}

    ngOnInit(): void {
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

        const currentUrl = this.router.url;
        if (mode === 'REAL' && currentUrl !== '/real-mode') {
            this.router.navigate(['/real-mode']).finally(() => (this.syncing = false));
            return;
        }

        if (mode === 'SIMULATION' && currentUrl !== '/simulation-mode') {
            this.router.navigate(['/simulation-mode']).finally(() => (this.syncing = false));
            return;
        }

        if (mode === null && (currentUrl === '/simulation-mode' || currentUrl === '/real-mode')) {
            this.router.navigate(['/home']).finally(() => (this.syncing = false));
            return;
        }

        this.syncing = false;
    }
}
