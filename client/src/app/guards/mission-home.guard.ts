import { inject } from '@angular/core';
import { CanActivateFn, Router } from '@angular/router';
import { MissionModeService } from '@app/services/mission-mode/mission-mode.service';

export const missionHomeGuard: CanActivateFn = () => {
    const missionModeService = inject(MissionModeService);
    const router = inject(Router);

    return missionModeService
        .ensureModeLoaded()
        .then((mode) => {
            if (!mode) {
                return true;
            }

            if (mode === 'REAL') {
                router.navigate(['/real-mode']);
            } else if (mode === 'SIMULATION') {
                router.navigate(['/simulation-mode']);
            }
            return false;
        })
        .catch(() => true);
};
