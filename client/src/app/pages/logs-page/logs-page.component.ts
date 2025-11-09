import { Component, OnInit } from '@angular/core';
import { CommonModule } from '@angular/common';
import { FormsModule } from '@angular/forms';
import { MissionListComponent } from '@app/components/mission-list/mission-list.component';
import { ActivatedRoute } from '@angular/router';

interface LogDetails {
  [key: string]: any; // pour permettre l’indexation dynamique
}

interface MissionLogEntry {
  timestamp: string;
  robot: string;
  category: 'Command' | 'Sensor';
  action: string;
  details: LogDetails;
}

@Component({
  selector: 'app-logs-page',
  standalone: true,
  imports: [CommonModule, FormsModule, MissionListComponent],
  templateUrl: './logs-page.component.html',
  styleUrls: ['./logs-page.component.scss'],
})
export class LogsPageComponent implements OnInit {
  activeTab: 'live' | 'history' = 'live';

  missionId: string | null = null;

  missionName: string | null = null;

  // données samples pour le design de l’UI
  liveData: MissionLogEntry[] = [
    {
      timestamp: '18:12:03',
      robot: 'limo1',
      category: 'Command',
      action: 'start_mission',
      details: { note: 'Initialisation' },
    },
    {
      timestamp: '18:12:05',
      robot: 'limo2',
      category: 'Sensor',
      action: 'odom',
      details: { x: 0.21, y: 1.02, distance: 3.85, orientation: 90 },
    },
    {
      timestamp: '18:12:05',
      robot: 'limo2',
      category: 'Sensor',
      action: 'lidar',
      details: { min: 0.4, angle: 45 },
    },
  ];

  constructor(private route: ActivatedRoute) {}

  ngOnInit(): void {
    this.route.queryParamMap.subscribe((params) => {
      this.missionId = params.get('missionId');
      this.missionName = params.get('missionName');
    });
  }

  get hasActiveMission(): boolean {
    return !!(this.missionId || this.missionName);
  }

  switchTab(tab: 'live' | 'history') {
    this.activeTab = tab;
  }

  objectKeys = Object.keys;
}
