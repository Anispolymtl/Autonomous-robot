import { Component, OnDestroy, OnInit } from '@angular/core';
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
import { Point2D } from '@app/components/map/map.component';

type RobotId = 'limo1' | 'limo2';
type MissedWaypointPayload = { missedPoints?: number[]; originalWaypoints?: Point2D[] };

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
export class SimulationPageComponent implements OnInit, OnDestroy {
  message: string | null = null;
  selectedRobotId: RobotId = 'limo1';
  private missedWaypointHandlers: Partial<Record<RobotId, (...args: any[]) => void>> = {};

  constructor(
    private router: Router,
    private missonService: MissionService,
    private missionSessionService: MissionSessionService,
    private socketService: SocketService,
    private dialog: MatDialog
  ) { }

  ngOnInit(): void {
    void this.missionSessionService.rehydrateActiveMission();
    this.ensureSocketListeners();
  }

  ngOnDestroy(): void {
    (['limo1', 'limo2'] as RobotId[]).forEach((robotId) => {
      const handler = this.missedWaypointHandlers[robotId];
      if (handler) this.socketService.off(`/${robotId}/missedWaypoints`, handler);
    });
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

  private ensureSocketListeners(): void {
    if (!this.socketService.isSocketAlive()) {
      this.socketService.connect('client');
      const socket = this.socketService.getSocket;
      if (socket) {
        socket.once('connect', () => this.registerMissedWaypointListeners());
      }
    } else {
      this.registerMissedWaypointListeners();
    }
  }

  private registerMissedWaypointListeners(): void {
    (['limo1', 'limo2'] as RobotId[]).forEach((robotId) => {
      const handler = (...args: any[]) => this.handleMissedWaypoints(robotId, args[0] as MissedWaypointPayload);
      this.missedWaypointHandlers[robotId] = handler;
      this.socketService.on(`/${robotId}/missedWaypoints`, handler);
    });
  }

  private handleMissedWaypoints(robotId: RobotId, payload: MissedWaypointPayload): void {
    const missedPoints = Array.isArray(payload?.missedPoints) ? payload.missedPoints : [];
    const originalWaypoints = Array.isArray(payload?.originalWaypoints) ? payload.originalWaypoints : [];

    if (!missedPoints.length || !originalWaypoints.length) return;

    // Il n'y aura qu'un seul point dans missedPoints; on liste tous les waypoints à partir de cet index
    const startIndex = missedPoints[0];
    const missedList = originalWaypoints
      .map((waypoint, index) => ({ waypoint, index }))
      .filter(({ index }) => index >= startIndex)
      .map(({ waypoint, index }) => {
        const x = Number.isFinite(waypoint?.x) ? waypoint.x.toFixed(2) : '?';
        const y = Number.isFinite(waypoint?.y) ? waypoint.y.toFixed(2) : '?';
        return `#${index} (${x}, ${y})`;
      });

    if (!missedList.length) return;

    this.dialog.open(ConfirmationDialogComponent, {
      width: '520px',
      data: {
        title: 'Trajectoire non suivie',
        message: `${robotId} n'a pu suivre la trajectoire. Il n'a pas pu aller aux points:`,
        consequences: [...missedList, 'Veuillez définir une nouvelle trajectoire.'],
        confirmText: 'OK',
        tone: 'danger',
      },
    });
  }
}
