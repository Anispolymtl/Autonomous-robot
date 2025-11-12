import { inject } from '@angular/core';
import { CanActivateFn, Router } from '@angular/router';
import { MissionModeService, MissionMode } from '@app/services/mission-mode/mission-mode.service';

export function missionModeGuard(requiredMode: Exclude<MissionMode, null>): CanActivateFn {
    return () => {
        const missionModeService = inject(MissionModeService);
        const router = inject(Router);

        return missionModeService.ensureModeLoaded().then((mode) => {
            if (mode === requiredMode) {
                return true;
            }

            if (mode === 'REAL') {
                router.navigate(['/real-mode']);
            } else if (mode === 'SIMULATION') {
                router.navigate(['/simulation-mode']);
            } else {
                router.navigate(['/home']);
            }
            return false;
        }).catch(() => true);
    };
}
