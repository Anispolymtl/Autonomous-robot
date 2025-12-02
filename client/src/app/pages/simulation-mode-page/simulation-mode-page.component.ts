import { Component, OnInit } from '@angular/core';
import { CommonModule } from '@angular/common';
import { Router } from '@angular/router';
import { MissionService } from '@app/services/mission/mission.service';
import { HttpErrorResponse } from '@angular/common/http';
import { MapComponent } from '@app/components/map/map.component';
import { RobotStatusComponent } from '@app/components/robot-status/robot-status.component';
import { SocketService } from '@app/services/socket/socket.service';
import { MissionSessionService } from '@app/services/mission-session/mission-session.service';
import { MergedMapComponent } from '@app/components/merged-map/merged-map.component';
import { MatDialog } from '@angular/material/dialog';
import { CodeEditorDialogComponent } from '@app/components/code-editor-dialog/code-editor-dialog.component';
import { ConfirmationDialogComponent } from '@app/components/confirmation-dialog/confirmation-dialog.component';

type RobotId = 'limo1' | 'limo2';

@Component({
  selector: 'app-simulation-page',
  standalone: true,
  imports: [
    MapComponent,
    CommonModule,
    RobotStatusComponent,
    MergedMapComponent
  ],
  templateUrl: './simulation-mode-page.component.html',
  styleUrls: ['./simulation-mode-page.component.scss'],
})
export class SimulationPageComponent implements OnInit {
  message: string | null = null;
  selectedRobotId: RobotId = 'limo1';

  constructor(
    private router: Router,
    private missonService: MissionService,
    private missionSessionService: MissionSessionService,
    private socketService: SocketService,
    private dialog: MatDialog
  ) { }

  ngOnInit(): void {
    void this.missionSessionService.rehydrateActiveMission();
  }

  startMission(): void {
    this.message = 'Mission demandée.';
    console.log('Mission demandée');
    this.missionSessionService.markMissionStarted();
    this.missonService.startMission().subscribe({
      next: () => {
        console.log('Mission started successfully');
      },
      error: (error: HttpErrorResponse) => {
        console.error('Error starting mission:', error);
      }
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
          'L’arrêt immédiat des robots.',
          'L’interruption de la navigation en cours.',
          'La sauvegarde et la clôture de la mission.',
          'Un retour à l’accueil.',
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
      this.missonService.cancelMission().subscribe({
        next: () => {
          console.log('Mission stopped successfully');
          this.finalizeMission();
        },
        error: (error: HttpErrorResponse) => {
          console.error('Error stopping mission:', error);
          this.finalizeMission();
        }
      });
    });
  }

  navigateToEditPage(): void {
    this.dialog.open(CodeEditorDialogComponent, {
      width: '85vw',
      height: '85vh',
      panelClass: 'code-editor-dialog'
    });
  }

  private finalizeMission(): void {
    this.missionSessionService.completeMission()
      .then(() => {
        this.router.navigate(['/home']);
      })
      .catch((error) => {
        console.error('Erreur lors de la finalisation de la mission:', error);
        this.router.navigate(['/home']);
      });
  }

  setSelectedRobot(robotId: RobotId): void {
    this.selectedRobotId = robotId;
  }

  returnToBase(): void {
    console.log('Retour à la base demandé pour tous les robots');
    this.message = 'Retour à la base en cours pour tous les robots...';
    this.socketService.send('nav:return-to-base');
  }
}
