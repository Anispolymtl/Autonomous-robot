import { Component } from '@angular/core';
import { CommonModule } from '@angular/common';
import { FormsModule } from '@angular/forms';
import { Router } from '@angular/router';
import { MissionSessionService } from '@app/services/mission-session/mission-session.service';

type ModeType = 'simulation' | 'real';

@Component({
  selector: 'app-robot-login',
  standalone: true,
  imports: [CommonModule, FormsModule],
  templateUrl: './robot-login.component.html',
  styleUrls: ['./robot-login.component.scss'],
})
export class RobotLoginComponent {
  selectedMode: ModeType | null = null;
  missionName = '';
  missionNameTouched = false;
  error: string | null = null;

  constructor(
    private router: Router,
    private missionSessionService: MissionSessionService
  ) {}

  selectMode(mode: ModeType): void {
    this.selectedMode = mode;
  }

  get isMissionNameValid(): boolean {
    return this.missionName.trim().length > 0;
  }

  onMissionNameBlur(): void {
    this.missionNameTouched = true;
  }

  async onModeSubmit(): Promise<void> {
    this.missionNameTouched = true;
    this.error = null;
    if (!this.selectedMode || !this.isMissionNameValid) return;

    try {
      await this.missionSessionService.initializeMission(
        this.missionName.trim(),
        this.selectedMode === 'real' ? 'REAL' : 'SIMULATION'
      );
      const targetRoute = this.selectedMode === 'simulation' ? '/simulation-mode' : '/real-mode';
      this.router.navigate([targetRoute]);
    } catch (err) {
      this.error = (err as Error)?.message ?? 'Impossible de créer la mission.';
    }
  }
}
