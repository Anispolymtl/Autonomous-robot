import { Component } from '@angular/core';
import { CommonModule } from '@angular/common';
import { FormsModule } from '@angular/forms';

interface Mission {
  id: number;
  name: string;
  date: string;
  duration: string;
  robots: number;
  status: string;
  size: string;
}

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
  imports: [CommonModule, FormsModule],
  templateUrl: './logs-page.component.html',
  styleUrls: ['./logs-page.component.scss'],
})
export class LogsPageComponent {
  activeTab: 'live' | 'history' | 'analytics' = 'live';
  selectedMission: Mission | null = null;
  expandedRobot: number | null = null;
  searchQuery: string = '';

  // Données simulées
  missions: Mission[] = [
    { 
      id: 1, 
      name: 'Mission Alpha - Scan Sector 7', 
      date: '2025-10-15 14:32', 
      duration: '45min', 
      robots: 3, 
      status: 'completed', 
      size: '2.4GB' 
    },
    { 
      id: 2, 
      name: 'Mission Beta - Reconnaissance', 
      date: '2025-10-14 10:15', 
      duration: '1h 12min', 
      robots: 5, 
      status: 'completed', 
      size: '4.1GB' 
    },
    { 
      id: 3, 
      name: 'Mission Gamma - Perimeter Check', 
      date: '2025-10-13 09:20', 
      duration: '32min', 
      robots: 2, 
      status: 'completed', 
      size: '1.8GB' 
    },
  ];

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

  get filteredMissions(): Mission[] {
    if (!this.searchQuery.trim()) {
      return this.missions;
    }
    const query = this.searchQuery.toLowerCase();
    return this.missions.filter(m => 
      m.name.toLowerCase().includes(query) ||
      m.date.includes(query)
    );
  }

  switchTab(tab: 'live' | 'history' | 'analytics') {
    this.activeTab = tab;
  }

  selectMission(mission: Mission) {
    this.selectedMission = mission;
    this.activeTab = 'history';
  }

  toggleRobot(id: number) {
    this.expandedRobot = this.expandedRobot === id ? null : id;
  }

  downloadLogs() {
    console.log('Téléchargement des logs pour:', this.selectedMission?.name);
    alert(`Téléchargement des logs: ${this.selectedMission?.name}`);
  }
}