import { Component } from '@angular/core';
import { CommonModule } from '@angular/common';
import { Router } from '@angular/router';
import { MissionService } from '@app/services/mission.service';
import { HttpErrorResponse } from '@angular/common/http';
import { MapComponent } from '@app/components/map/map.component';
import { RobotStatusComponent } from '@app/components/robot-status/robot-status.component';

type RobotId = 'limo1' | 'limo2';

@Component({
  selector: 'app-simulation-page',
  standalone: true,
  imports: [
      MapComponent,
      CommonModule,
      RobotStatusComponent
  ],
  templateUrl: './simulation-mode-page.component.html',
  styleUrls: ['./simulation-mode-page.component.scss'],
})
export class SimulationPageComponent {
  message: string | null = null;
  selectedRobotId: RobotId = 'limo1';

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

  setSelectedRobot(robotId: RobotId): void {
    this.selectedRobotId = robotId;
  }

  back(): void {
    this.router.navigate(['/home']);
  }
}
