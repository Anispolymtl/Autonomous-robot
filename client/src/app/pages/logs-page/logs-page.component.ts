import { Component, OnInit } from '@angular/core';
import { CommonModule } from '@angular/common';
import { FormsModule } from '@angular/forms';
import { MissionListComponent } from '@app/components/mission-list/mission-list.component';
import { ActivatedRoute } from '@angular/router';

interface LiveData {
  robot: string;
  timestamp: string;
  distance: number;
  x: number;
  y: number;
  command: string;
  status: string;
}

@Component({
  selector: 'app-logs-page',
  standalone: true,
  imports: [CommonModule, FormsModule, MissionListComponent],
  templateUrl: './logs-page.component.html',
  styleUrls: ['./logs-page.component.scss'],
})
export class LogsPageComponent implements OnInit {
  activeTab: 'live' | 'history' | 'analytics' = 'live';
  expandedRobot: number | null = null;
  missionId: string | null = null;

  liveData: LiveData[] = [
    { 
      robot: 'Robot-01', 
      timestamp: '14:32:45', 
      distance: 2.34, 
      x: 156.2, 
      y: 89.3, 
      command: 'MOVE_FORWARD', 
      status: 'executing' 
    },
    { 
      robot: 'Robot-02', 
      timestamp: '14:32:45', 
      distance: 1.12, 
      x: 142.1, 
      y: 101.5, 
      command: 'ROTATE_LEFT', 
      status: 'executing' 
    },
    { 
      robot: 'Robot-03', 
      timestamp: '14:32:44', 
      distance: 0.89, 
      x: 168.9, 
      y: 95.2, 
      command: 'IDLE', 
      status: 'idle' 
    },
  ];

  robots = [1, 2, 3];

  constructor(private route: ActivatedRoute) {}

  ngOnInit(): void {
    this.route.queryParamMap.subscribe((params) => {
      this.missionId = params.get('missionId');
    });
  }

  switchTab(tab: 'live' | 'history' | 'analytics') {
    this.activeTab = tab;
  }

  toggleRobot(id: number) {
    this.expandedRobot = this.expandedRobot === id ? null : id;
  }
}
