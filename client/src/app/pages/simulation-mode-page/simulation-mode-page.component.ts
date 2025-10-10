import { Component } from '@angular/core';
import { CommonModule } from '@angular/common';
import { Router } from '@angular/router';
import { MissionService } from 'src/app/services/mission/mission.service';
import { HttpErrorResponse } from '@angular/common/http';
import { MapGridComponent } from '@app/components/map-grid/map-grid.component';

@Component({
  selector: 'app-simulation-page',
  standalone: true,
  imports: [
      MapGridComponent,
      CommonModule
  ],
  templateUrl: './simulation-mode-page.component.html',
  styleUrls: ['./simulation-mode-page.component.scss'],
})
export class SimulationPageComponent {
  message: string | null = null;
  constructor(private router: Router, private missonService: MissionService) { }

  startMission(): void {
    this.message = 'Mission demandée.';
    console.log('Mission demandée');
    this.missonService.startMission().subscribe({
      next: (response: any) => {
        console.log('Mission started successfully:', response);
      },
      error: (error: HttpErrorResponse) => {
        console.error('Error starting mission:', error);
      }
    });
  }

  stopMission(): void {
    this.message = 'Mission terminée.';
    console.log('Mission terminée');
    this.missonService.cancelMission().subscribe({
      next: (response: any) => {
        console.log('Mission stopped successfully:', response);
      },
      error: (error: HttpErrorResponse) => {
        console.error('Error stopping mission:', error);
      }
    });
  }

  back(): void {
    this.router.navigate(['/home']);
  }
}