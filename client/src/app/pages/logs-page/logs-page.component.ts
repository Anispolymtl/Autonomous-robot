import { Component, OnDestroy, OnInit } from '@angular/core';
import { CommonModule } from '@angular/common';
import { FormsModule } from '@angular/forms';
import { MissionListComponent } from '@app/components/mission-list/mission-list.component';
import { ActivatedRoute } from '@angular/router';
import { Mission, MissionLogEntry } from '@app/interfaces/mission';
import { MissionModeService } from '@app/services/mission-mode.service';
import { SocketService } from '@app/services/socket.service';
import { Subscription, firstValueFrom } from 'rxjs';

interface MissionEventPayload {
  missionId: string;
  mission: Mission;
}

@Component({
  selector: 'app-logs-page',
  standalone: true,
  imports: [CommonModule, FormsModule, MissionListComponent],
  templateUrl: './logs-page.component.html',
  styleUrls: ['./logs-page.component.scss'],
})

export class LogsPageComponent implements OnInit, OnDestroy {
  activeTab: 'live' | 'history' = 'live';

  missionId: string | null = null;

  missionName: string | null = null;

  liveData: MissionLogEntry[] = [];
  private missionSub?: Subscription;
  private socketListenersRegistered = false;

  private missionUpdateHandler = (...args: unknown[]) => {
    const payload = args[0] as MissionEventPayload | undefined;
    if (!payload || !this.missionId || payload.missionId !== this.missionId) return;
    this.liveData = this.sortLogs(payload.mission.logs ?? []);
    this.missionName = payload.mission.missionName ?? this.missionName;
  };

  private missionFinalizedHandler = (...args: unknown[]) => {
    const payload = args[0] as MissionEventPayload | undefined;
    if (!payload || !this.missionId || payload.missionId !== this.missionId) return;
    this.liveData = this.sortLogs(payload.mission.logs ?? []);
    this.missionId = null;
    this.missionName = null;
    this.cleanupSocketListeners();
  };

  constructor(
    private route: ActivatedRoute,
    private missionModeService: MissionModeService,
    private socketService: SocketService
  ) {}

  ngOnInit(): void {
    this.missionSub = this.route.queryParamMap.subscribe((params) => {
      const routeMissionId = params.get('missionId');
      const routeMissionName = params.get('missionName');
      if (routeMissionId) this.missionId = routeMissionId;
      if (routeMissionName) this.missionName = routeMissionName;
    });
    this.loadActiveMission();
  }

  ngOnDestroy(): void {
    this.missionSub?.unsubscribe();
    this.cleanupSocketListeners();
  }

  get hasActiveMission(): boolean {
    return !!(this.missionId || this.missionName);
  }

  switchTab(tab: 'live' | 'history') {
    this.activeTab = tab;
  }

  objectKeys = Object.keys;

  private async loadActiveMission(): Promise<void> {
    try {
      const mission = await firstValueFrom(this.missionModeService.fetchActiveMission());
      if (!mission) {
        this.liveData = [];
        this.missionId = null;
        this.missionName = null;
        this.cleanupSocketListeners();
        return;
      }

      this.missionId = mission.missionId;
      this.missionName = mission.missionName ?? this.missionName;
      this.liveData = this.sortLogs(mission.logs ?? []);
      await this.ensureSocketConnected();
      this.registerSocketListeners();
    } catch (error) {
      console.error('Erreur lors du chargement de la mission active:', error);
      this.liveData = [];
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
