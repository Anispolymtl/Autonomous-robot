import { Component } from '@angular/core';
import { CommonModule } from '@angular/common';
import { FormsModule } from '@angular/forms';
import { Router } from '@angular/router';

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

  constructor(private router: Router) {}

  selectMode(mode: ModeType): void {
    this.selectedMode = mode;
  }

  get isMissionNameValid(): boolean {
    return this.missionName.trim().length > 0;
  }

  onMissionNameBlur(): void {
    this.missionNameTouched = true;
  }

  onModeSubmit(): void {
    this.missionNameTouched = true;
    if (!this.selectedMode || !this.isMissionNameValid) return;

    const targetRoute = this.selectedMode === 'simulation' ? '/simulation-mode' : '/real-mode';
    this.router.navigate([targetRoute], {
      state: { missionName: this.missionName.trim(), mode: this.selectedMode }
    });
  }
}
