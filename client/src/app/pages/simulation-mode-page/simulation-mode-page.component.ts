import { Component, OnInit, OnDestroy } from '@angular/core';
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
import { MissionModeService } from '@app/services/mission-mode/mission-mode.service';
import { Mission, MissionLogEntry } from '@app/interfaces/mission';
import { Subscription, firstValueFrom } from 'rxjs';

type RobotId = 'limo1' | 'limo2';

interface MissionEventPayload {
  missionId: string;
  mission: Mission;
}

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
  
  // Logs sidebar state
  isLogsSidebarOpen = false;
  missionId: string | null = null;
  missionName: string | null = null;
  liveLogsData: MissionLogEntry[] = [];
  
  private missionSub?: Subscription;
  private socketListenersRegistered = false;

  private missionUpdateHandler = (...args: unknown[]) => {
    const payload = args[0] as MissionEventPayload | undefined;
    if (!payload || !this.missionId || payload.missionId !== this.missionId) return;
    this.liveLogsData = this.sortLogs(payload.mission.logs ?? []);
    this.missionName = payload.mission.missionName ?? this.missionName;
  };

  private missionFinalizedHandler = (...args: unknown[]) => {
    const payload = args[0] as MissionEventPayload | undefined;
    if (!payload || !this.missionId || payload.missionId !== this.missionId) return;
    this.liveLogsData = this.sortLogs(payload.mission.logs ?? []);
  };

  constructor(
    private router: Router,
    private missonService: MissionService,
    private missionSessionService: MissionSessionService,
    private socketService: SocketService,
    private dialog: MatDialog,
    private missionModeService: MissionModeService
  ) { }

  ngOnInit(): void {
    this.missionSessionService.rehydrateActiveMission();
    this.loadActiveMission();
  }

  ngOnDestroy(): void {
    this.missionSub?.unsubscribe();
    this.cleanupSocketListeners();
  }

  toggleLogsSidebar(): void {
    this.isLogsSidebarOpen = !this.isLogsSidebarOpen;
  }

  objectKeys = Object.keys;
  detailEntries = (details: any): { key: string; value: any }[] =>
    details && typeof details === 'object'
      ? Object.keys(details).map((key) => ({ key, value: (details as any)[key] }))
      : [];

  formatTime = (timestamp: string): string => {
    try {
      const date = new Date(timestamp);
      return date.toLocaleTimeString('fr-CA', { hour: '2-digit', minute: '2-digit', second: '2-digit' });
    } catch {
      return timestamp;
    }
  };

  startMission(): void {
    this.message = 'Mission demandée.';
    console.log('Mission demandée');
    this.missionSessionService.markMissionStarted();
    this.missonService.startMission().subscribe({
      next: (response: any) => {
        console.log('Mission started successfully:', response);
        this.loadActiveMission();
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

  navigateToEditPage(): void {
    this.dialog.open(CodeEditorDialogComponent, {
      width: '85vw',
      height: '85vh',
      panelClass: 'code-editor-dialog'
    });
  }

  private finalizeMission(): void {
    this.missionSessionService.completeMission()
      .then((mission) => {
        if (!mission) {
          console.log('storing error')
          this.router.navigate(['/home']);
        } else {
          this.router.navigate(['/home']);
        }
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

  identifyRobot(robotId: RobotId): void {
    console.log(`Identification demandée pour ${robotId}`);
    this.message = `Identification de ${robotId === 'limo1' ? 'Limo 1' : 'Limo 2'}...`;
  }

  // Logs methods
  private async loadActiveMission(): Promise<void> {
    try {
      const mission = await firstValueFrom(this.missionModeService.fetchActiveMission());
      if (!mission) {
        this.liveLogsData = [];
        this.missionId = null;
        this.missionName = null;
        this.cleanupSocketListeners();
        return;
      }

      this.missionId = mission.missionId;
      this.missionName = mission.missionName ?? this.missionName;
      this.liveLogsData = this.sortLogs(mission.logs ?? []);
      await this.ensureSocketConnected();
      this.registerSocketListeners();
    } catch (error) {
      console.error('Erreur lors du chargement de la mission active:', error);
      this.liveLogsData = [];
    }
  }

  private async ensureSocketConnected(): Promise<void> {
    if (this.socketService.isSocketAlive()) return;
    this.socketService.connect('client');
    await new Promise<void>((resolve) => {
      if (this.socketService.isSocketAlive()) {
        resolve();
        return;
      }
      this.socketService.once('connect', () => resolve());
    });
  }

  private registerSocketListeners(): void {
    if (this.socketListenersRegistered) return;
    this.socketService.on('mission:updated', this.missionUpdateHandler);
    this.socketService.on('mission:finalized', this.missionFinalizedHandler);
    this.socketListenersRegistered = true;
  }

  private cleanupSocketListeners(): void {
    if (!this.socketListenersRegistered) return;
    this.socketService.off('mission:updated', this.missionUpdateHandler);
    this.socketService.off('mission:finalized', this.missionFinalizedHandler);
    this.socketListenersRegistered = false;
  }

  private sortLogs(logs: MissionLogEntry[]): MissionLogEntry[] {
    return [...logs].sort(
      (a, b) => new Date(b.timestamp).getTime() - new Date(a.timestamp).getTime()
    );
  }
}
