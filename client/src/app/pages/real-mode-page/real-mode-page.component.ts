import { Component, OnInit, ViewChild } from '@angular/core';
import { CommonModule } from '@angular/common';
import { Router } from '@angular/router';
import { HttpErrorResponse } from '@angular/common/http';
import { IdentifyService } from '@app/services/identify/identify.service';
import { MissionService } from '@app/services/mission/mission.service';
import { RobotStatusComponent } from '@app/components/robot-status/robot-status.component';
import { MissionSessionService } from '@app/services/mission-session/mission-session.service';
import { MissionDatabaseService } from '@app/services/mission-database/mission-database.service';
import { MergedMapComponent } from '@app/components/merged-map/merged-map.component';
import { SocketService } from '@app/services/socket/socket.service';
import { MatDialog } from '@angular/material/dialog';
import { ConfirmationDialogComponent } from '@app/components/confirmation-dialog/confirmation-dialog.component';

type RobotId = 'limo1' | 'limo2';

@Component({
  selector: 'app-real-page',
  standalone: true,
  imports: [RobotStatusComponent, CommonModule, MergedMapComponent],
  templateUrl: './real-mode-page.component.html',
  styleUrls: ['./real-mode-page.component.scss'],
})
export class RealPageComponent implements OnInit {
  message: string | null = null;

  @ViewChild(RobotStatusComponent)
  robotStatusComponent!: RobotStatusComponent;

  constructor(
    private router: Router,
    private identifyService: IdentifyService,
    private missionService: MissionService,
    private missionSessionService: MissionSessionService,
    private missionDatabaseService: MissionDatabaseService,
    private socketService: SocketService,
    private dialog: MatDialog
  ) {}

  ngOnInit(): void {
    void this.missionSessionService.rehydrateActiveMission();
  }

  identifyRobot(robotId: RobotId): void {
    const robotNumber = robotId === 'limo2' ? 2 : 1;
    this.identifyService.identifyRobot(robotNumber).subscribe({
      next: (res: any) => {
        this.message = `Réponse du robot ${robotNumber} : ${res.message}`;
        this.missionSessionService.appendLog({
          category: 'Command',
          robot: robotId,
          action: 'identify_robot',
          details: { success: true, message: res?.message ?? null },
        });
      },
      error: (err: HttpErrorResponse) => {
        this.message = `Erreur lors de l'identification du robot ${robotNumber} : ${err.message}`;
        this.missionSessionService.appendLog({
          category: 'Command',
          robot: robotId,
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
    const dialogRef = this.dialog.open(ConfirmationDialogComponent, {
      width: '520px',
      data: {
        title: 'Terminer la mission',
        message:
          'Voulez-vous vraiment terminer la mission ? Cette action entraîne :',
        consequences: [
          'Arrêt immédiat de tous les robots.',
          'Annulation de la navigation ou de l’exploration en cours.',
          'Clôture et sauvegarde de la mission, puis retour à l’accueil.',
        ],
        confirmText: 'Terminer la mission',
        cancelText: 'Continuer la mission',
        tone: 'danger',
      },
    });

    dialogRef.afterClosed().subscribe((confirmed) => {
      if (!confirmed) return;
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
    });
  }

  returnToBase(): void {
    this.message = 'Retour à la base en cours pour tous les robots...';
    this.socketService.send('nav:return-to-base');
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
