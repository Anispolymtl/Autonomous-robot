import { Component, OnInit, ViewChild } from '@angular/core';
import { CommonModule } from '@angular/common';
import { Router } from '@angular/router';
import { ReactiveFormsModule, FormGroup } from '@angular/forms';
import { HttpErrorResponse } from '@angular/common/http';
import { IdentifyService } from '@app/services/identify/identify.service';
import { MissionService } from '@app/services/mission/mission.service';
import { MapComponent } from '@app/components/map/map.component';
import { RobotStatusComponent } from '@app/components/robot-status/robot-status.component';
import { MissionSessionService } from '@app/services/mission-session/mission-session.service';
import { MissionDatabaseService } from '@app/services/mission-database/mission-database.service';

type RobotId = 'limo1' | 'limo2';

@Component({
  selector: 'app-real-page',
  standalone: true,
  imports: [MapComponent, RobotStatusComponent, CommonModule, ReactiveFormsModule],
  templateUrl: './real-mode-page.component.html',
  styleUrls: ['./real-mode-page.component.scss'],
})
export class RealPageComponent implements OnInit {
  form: FormGroup;
  message: string | null = null;
  selectedRobotId: RobotId = 'limo1';


  @ViewChild(RobotStatusComponent)
  robotStatusComponent!: RobotStatusComponent;

  constructor(
    private router: Router,
    private identifyService: IdentifyService,
    private missionService: MissionService,
    private missionSessionService: MissionSessionService,
    private missionDatabaseService: MissionDatabaseService
  ) {}

  ngOnInit(): void {
    this.missionSessionService.rehydrateActiveMission();
  }

  onIdentify(robotId: number): void {
    const robotNamespace = this.asRobotNamespace(robotId);
    this.identifyService.identifyRobot(robotId).subscribe({
      next: (res: any) => {
        this.message = `Réponse du robot ${robotId} : ${res.message}`;
        this.missionSessionService.appendLog({
          category: 'Command',
          robot: robotNamespace,
          action: 'identify_robot',
          details: { success: true, message: res?.message ?? null },
        });
      },
      error: (err: HttpErrorResponse) => {
        this.message = `Erreur lors de l'identification du robot ${robotId} : ${err.message}`;
        this.missionSessionService.appendLog({
          category: 'Command',
          robot: robotNamespace,
          action: 'identify_robot',
          details: { success: false, error: err?.message ?? 'unknown' },
        });
      },
    });
  }

  startMission(): void {
    this.message = 'Mission demandée.';
    console.log('Mission demandée');
    this.missionSessionService.markMissionStarted();
    this.missionService.startMission().subscribe({
      next: (response: any) => {
        console.log('Mission started successfully:', response);
      },
      error: (error: HttpErrorResponse) => {
        console.error('Error starting mission:', error);
      },
    });
  }

  stopMission(): void {
    this.message = 'Mission terminée.';
    console.log('Mission terminée');
    this.missionService.cancelMission().subscribe({
      next: (response: any) => {
        console.log('Mission stopped successfully:', response);
        this.finalizeMission();
      },
      error: (error: HttpErrorResponse) => {
        console.error('Error stopping mission:', error);
        this.finalizeMission();
      },
    });
  }

  setSelectedRobot(robotId: RobotId): void {
    this.selectedRobotId = robotId;
  }

  private asRobotNamespace(robotId: number): RobotId {
    return robotId === 2 ? 'limo2' : 'limo1';
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
          },
        });
      })
      .catch((error) => {
        console.error('Erreur lors de la finalisation de la mission:', error);
        this.router.navigate(['/home']);
      });
  }
}
