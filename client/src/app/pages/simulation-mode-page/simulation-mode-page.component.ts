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
  template: `
<div class="simulation-page">
  <!-- Navigation Bar -->
  <header class="top-navbar">
    <div class="navbar-content">
      <div class="navbar-left">
        <button class="icon-btn" title="Menu">
          <svg width="24" height="24" viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2">
            <line x1="3" y1="12" x2="21" y2="12"></line>
            <line x1="3" y1="6" x2="21" y2="6"></line>
            <line x1="3" y1="18" x2="21" y2="18"></line>
          </svg>
        </button>
        <h1 class="app-title">Mission de Simulation</h1>
      </div>
      <div class="navbar-right">
        <button class="icon-btn logs-toggle-btn" (click)="toggleLogsSidebar()" title="Afficher/Masquer les Logs">
          <svg width="24" height="24" viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2">
            <path d="M14 2H6a2 2 0 0 0-2 2v16a2 2 0 0 0 2 2h12a2 2 0 0 0 2-2V8z"></path>
            <polyline points="14 2 14 8 20 8"></polyline>
            <line x1="16" y1="13" x2="8" y2="13"></line>
            <line x1="16" y1="17" x2="8" y2="17"></line>
          </svg>
          <span class="logs-badge" *ngIf="liveLogsData.length > 0">{{ liveLogsData.length }}</span>
        </button>
        <span class="mode-badge">Mode: Simulation</span>
      </div>
    </div>
  </header>

  <div class="main-container" [class.logs-open]="isLogsSidebarOpen">
    <!-- Sidebar gauche - Contrôles -->
    <aside class="sidebar sidebar-left">
      <div class="editor-fab-block">
        <button class="editor-fab" (click)="navigateToEditPage()" title="Modifier le code">
          <svg width="24" height="24" viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2">
            <path d="M12 19l7-7 3 3-7 7-3-3z"></path>
            <path d="M18 13l-1.5-7.5L2 2l3.5 14.5L13 18l5-5z"></path>
            <path d="M2 2l7.586 7.586"></path>
            <circle cx="11" cy="11" r="2"></circle>
          </svg>
        </button>
        <span class="editor-caption">Éditeur de code</span>
      </div>

      <!-- État des Robots -->
      <section class="control-section robot-status-section">
        <div class="section-header">
          <svg width="20" height="20" viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2">
            <rect x="2" y="7" width="20" height="14" rx="2" ry="2"></rect>
            <path d="M16 21V5a2 2 0 0 0-2-2h-4a2 2 0 0 0-2 2v16"></path>
          </svg>
          <h2>État des Robots</h2>
        </div>
        <app-robot-status 
          [mode]="'simulation'"
          (onReturnToBase)="returnToBase()"
          (onIdentifyRobot)="identifyRobot($event)">
        </app-robot-status>
      </section>

      <div class="alert alert-info" *ngIf="message">
        <svg width="18" height="18" viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2">
          <circle cx="12" cy="12" r="10"></circle>
          <line x1="12" y1="16" x2="12" y2="12"></line>
          <line x1="12" y1="8" x2="12.01" y2="8"></line>
        </svg>
        {{ message }}
      </div>
    </aside>

    <!-- Zone centrale - Cartes -->
    <main class="main-content">
      <!-- Mode Selection Card -->
      <section class="mode-selection-card">
        <div class="mode-card-header">
          <div class="mode-header-left">
            <svg width="20" height="20" viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2">
              <circle cx="12" cy="12" r="10"></circle>
              <path d="M12 16v-4"></path>
              <path d="M12 8h.01"></path>
            </svg>
            <h3>Choisissez votre mode d'exploration</h3>
          </div>
          <div class="mode-card-actions">
            <button class="btn btn-danger btn-sm" (click)="stopMission()">
              <svg width="16" height="16" viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2">
                <rect x="6" y="6" width="12" height="12"></rect>
              </svg>
              Arrêter la mission
            </button>
          </div>
        </div>
        <div class="mode-options">
          <div class="mode-option">
            <div class="mode-option-icon mode-icon-auto">
              <svg width="24" height="24" viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2">
                <polygon points="5 3 19 12 5 21 5 3"></polygon>
              </svg>
            </div>
            <div class="mode-option-content">
              <h4>Mission Autonome</h4>
              <p>Exploration automatique par les robots</p>
            </div>
            <button class="mode-option-action btn-primary" (click)="startMission()">
              Démarrer
            </button>
          </div>
          <div class="mode-divider">
            <span>OU</span>
          </div>
          <div class="mode-option">
            <div class="mode-option-icon mode-icon-manual">
              <svg width="24" height="24" viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2">
                <path d="M21 10c0 7-9 13-9 13s-9-6-9-13a9 9 0 0 1 18 0z"></path>
                <circle cx="12" cy="10" r="3"></circle>
              </svg>
            </div>
            <div class="mode-option-content">
              <h4>Trajectoire Manuelle</h4>
              <p>Cliquez sur la carte pour définir des points</p>
            </div>
            <div class="mode-option-hint">
              <svg width="16" height="16" viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2">
                <path d="M9 11l3 3L22 4"></path>
                <path d="M21 12v7a2 2 0 0 1-2 2H5a2 2 0 0 1-2-2V5a2 2 0 0 1 2-2h11"></path>
              </svg>
              <span>Voir la carte ci-dessous</span>
            </div>
          </div>
        </div>
      </section>

      <!-- Carte individuelle -->
      <section class="map-section">
        <div class="map-header">
          <div class="map-title">
            <svg width="20" height="20" viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2">
              <polygon points="1 6 1 22 8 18 16 22 23 18 23 2 16 6 8 2 1 6"></polygon>
              <line x1="8" y1="2" x2="8" y2="18"></line>
              <line x1="16" y1="6" x2="16" y2="22"></line>
            </svg>
            <h2>Carte d'Exploration Individuelle</h2>
          </div>
          <div class="robot-selector">
            <span class="selector-label">Robot:</span>
            <div class="btn-group">
              <button 
                class="btn btn-sm" 
                [class.btn-active]="selectedRobotId === 'limo1'"
                [disabled]="selectedRobotId === 'limo1'" 
                (click)="setSelectedRobot('limo1')">
                Limo 1
              </button>
              <button 
                class="btn btn-sm" 
                [class.btn-active]="selectedRobotId === 'limo2'"
                [disabled]="selectedRobotId === 'limo2'" 
                (click)="setSelectedRobot('limo2')">
                Limo 2
              </button>
            </div>
          </div>
        </div>
        <div class="map-content">
          <app-map *ngIf="selectedRobotId === 'limo1'" robotId="limo1"></app-map>
          <app-map *ngIf="selectedRobotId === 'limo2'" robotId="limo2"></app-map>
        </div>
      </section>

      <!-- Carte fusionnée -->
      <section class="map-section merged-map-section">
        <div class="map-header">
          <div class="map-title">
            <svg width="20" height="20" viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2">
              <path d="M21 16V8a2 2 0 0 0-1-1.73l-7-4a2 2 0 0 0-2 0l-7 4A2 2 0 0 0 3 8v8a2 2 0 0 0 1 1.73l7 4a2 2 0 0 0 2 0l7-4A2 2 0 0 0 21 16z"></path>
              <polyline points="7.5 4.21 12 6.81 16.5 4.21"></polyline>
              <polyline points="7.5 19.79 7.5 14.6 3 12"></polyline>
              <polyline points="21 12 16.5 14.6 16.5 19.79"></polyline>
              <polyline points="3.27 6.96 12 12.01 20.73 6.96"></polyline>
              <line x1="12" y1="22.08" x2="12" y2="12"></line>
            </svg>
            <h2>Carte Fusionnée</h2>
          </div>
        </div>
        <div class="map-content">
          <app-merged-map></app-merged-map>
        </div>
      </section>
    </main>

    <!-- Sidebar droite - Logs (Rétractable) -->
    <aside class="sidebar sidebar-logs" [class.open]="isLogsSidebarOpen">
      <div class="logs-sidebar-header">
        <div class="logs-header-top">
          <div class="logs-title">
            <svg width="20" height="20" viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2">
              <path d="M14 2H6a2 2 0 0 0-2 2v16a2 2 0 0 0 2 2h12a2 2 0 0 0 2-2V8z"></path>
              <polyline points="14 2 14 8 20 8"></polyline>
              <line x1="16" y1="13" x2="8" y2="13"></line>
              <line x1="16" y1="17" x2="8" y2="17"></line>
            </svg>
            <h2>Journal de Mission</h2>
          </div>
          <button class="icon-btn-small" (click)="toggleLogsSidebar()" title="Fermer">
            <svg width="20" height="20" viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2">
              <line x1="18" y1="6" x2="6" y2="18"></line>
              <line x1="6" y1="6" x2="18" y2="18"></line>
            </svg>
          </button>
        </div>
        <div class="mission-info" *ngIf="missionId">
          <span class="mission-label">Mission:</span>
          <span class="mission-value">{{ missionName || missionId }}</span>
        </div>
      </div>

      <div class="logs-content">
        <table class="logs-table">
          <thead>
            <tr>
              <th class="col-time">Heure</th>
              <th class="col-robot">Robot</th>
              <th class="col-details">Détails</th>
            </tr>
          </thead>
          <tbody>
            <tr *ngFor="let log of liveLogsData" class="log-row">
              <td class="col-time">
                <span class="time-badge">{{ formatTime(log.timestamp) }}</span>
              </td>
              <td class="col-robot">
                <div class="robot-cell">
                  <span class="robot-icon">🚗</span>
                  <span class="robot-name-log">{{ log.robot }}</span>
                </div>
              </td>
              <td class="col-details">
                <div class="details-container">
                  <ng-container *ngIf="detailEntries(log.details).length; else noDetails">
                    <span class="detail-tag" *ngFor="let entry of detailEntries(log.details)">
                      <span class="detail-key">{{ entry.key }}</span>
                      <span class="detail-value">{{ entry.value }}</span>
                    </span>
                  </ng-container>
                  <ng-template #noDetails>
                    <span class="detail-tag">
                      <span class="detail-key">Détails</span>
                      <span class="detail-value">—</span>
                    </span>
                  </ng-template>
                </div>
              </td>
            </tr>
          </tbody>
        </table>

        <!-- État vide -->
        <div class="empty-state" *ngIf="liveLogsData.length === 0">
          <div class="empty-icon">📭</div>
          <div class="empty-title">Aucun log disponible</div>
          <div class="empty-text">Les événements apparaîtront ici en temps réel</div>
        </div>
      </div>
    </aside>
  </div>
</div>
  `,
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
