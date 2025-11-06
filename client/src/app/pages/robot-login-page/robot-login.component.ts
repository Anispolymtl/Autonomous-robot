import { Component } from '@angular/core';
import { CommonModule } from '@angular/common';
import { Router } from '@angular/router';

type ModeType = 'simulation' | 'real';

@Component({
  selector: 'app-robot-login',
  standalone: true,
  imports: [CommonModule],
  templateUrl: './robot-login.component.html',
  styleUrls: ['./robot-login.component.scss'],
})
export class RobotLoginComponent {
  selectedMode: ModeType | null = null;

  constructor(private router: Router) {}

  selectMode(mode: ModeType): void {
    this.selectedMode = mode;
  }

  onModeSubmit(): void {
    if (!this.selectedMode) return;

    if (this.selectedMode === 'simulation') {
      this.router.navigate(['/simulation-mode']);
    } else {
      this.router.navigate(['/real-mode']);
    }
  }
}
