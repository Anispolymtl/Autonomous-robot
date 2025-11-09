import { Component, OnInit } from '@angular/core';
import { CommonModule } from '@angular/common';
import { Router } from '@angular/router';
import { MissionService } from '@app/services/mission.service';
import { HttpErrorResponse } from '@angular/common/http';
import { MapComponent } from '@app/components/map/map.component';
import { RobotStatusComponent } from '@app/components/robot-status/robot-status.component';
import { MissionSessionService } from '@app/services/mission-session.service';
import { MissionDatabaseService } from '@app/services/mission-database/mission-database.service';

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
export class SimulationPageComponent implements OnInit {
  message: string | null = null;
  constructor(
    private router: Router,
    private missonService: MissionService,
    private missionSessionService: MissionSessionService,
    private missionDatabaseService: MissionDatabaseService
  ) { }

  ngOnInit(): void {
    this.missionSessionService.rehydrateActiveMission();
  }

  startMission(): void {
    this.message = 'Mission demandée.';
    console.log('Mission demandée');
    this.missionSessionService.markMissionStarted();
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
        this.finalizeMission();
      },
      error: (error: HttpErrorResponse) => {
        console.error('Error stopping mission:', error);
        this.finalizeMission();
      }
    });
  }

  private finalizeMission(): void {
    this.missionSessionService.completeMission()
      .then((mission) => {
        if (!mission) {
          this.router.navigate(['/home']);
          return;
        }

        this.missionDatabaseService.createMission({
          missionName: mission.missionName,
          robots: mission.robots,
          mode: mission.mode,
          distance: mission.distance ?? 0,
          durationSec: mission.durationSec ?? 0,
          status: mission.status,
          logs: mission.logs ?? []
        }).subscribe({
          next: () => {
            console.log('Mission persistée en base de données');
            this.router.navigate(['/home']);
          },
          error: (err) => {
            console.error('Erreur lors de la sauvegarde de la mission:', err);
            this.router.navigate(['/home']);
          }
        });
      })
      .catch((error) => {
        console.error('Erreur lors de la finalisation de la mission:', error);
        this.router.navigate(['/home']);
      });
  }

  back(): void {
    this.router.navigate(['/home']);
  }
}
