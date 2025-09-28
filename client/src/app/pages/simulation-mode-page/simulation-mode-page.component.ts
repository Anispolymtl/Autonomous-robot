import { Component } from '@angular/core';
import { CommonModule } from '@angular/common';
import { Router } from '@angular/router';

@Component({
  selector: 'app-simulation-page',
  standalone: true,
  imports: [CommonModule],
  templateUrl: './simulation-mode-page.component.html',
  styleUrls: ['./simulation-mode-page.component.scss'],
})
export class SimulationPageComponent {
  message: string | null = null;
  constructor(private router: Router) { }

  startMission(): void {
    this.message = 'Mission simulation démarrée.';
    console.log('Mission simulation démarrée');
  }

  stopMission(): void {
    this.message = 'Mission simulation terminée.';
    console.log('Mission simulation terminée');
  }

  back(): void {
    this.router.navigate(['/robot-login']);
  }
}