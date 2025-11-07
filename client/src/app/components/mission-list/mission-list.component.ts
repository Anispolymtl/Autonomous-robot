import { Component, OnInit } from '@angular/core';
import { CommonModule } from '@angular/common';
import { FormsModule } from '@angular/forms';
import { Mission, MissionStats } from '@app/interfaces/mission';
import { HttpErrorResponse } from '@angular/common/http';
import { Observable } from 'rxjs';
import { MissionDatabaseService } from '@app/services/mission-database/mission-database.service';
import { MatDialog, MatDialogModule } from '@angular/material/dialog';
import { MissionLogsDialogComponent, MissionLogEntry } from '@app/components/mission-logs-dialog/mission-logs-dialog.component';

@Component({
    selector: 'app-mission-list',
    standalone: true,
    imports: [CommonModule, FormsModule, MatDialogModule],
    templateUrl: './mission-list.component.html',
    styleUrls: ['./mission-list.component.scss']
})
export class MissionListComponent implements OnInit {
    missions: Mission[] = [];
    stats: MissionStats | null = null;
    loading = false;
    error: string | null = null;
    selectedRobot: string | null = null;
    selectedMode: 'SIMULATION' | 'REAL' | null = null;

    constructor(
        private missionDatabaseService: MissionDatabaseService,
        private dialog: MatDialog
    ) { }

    ngOnInit(): void {
        this.loadMissions();
        this.loadStats();
    }

    loadMissions(): void {
        this.loading = true;
        this.error = null;
        
        let request: Observable<Mission[]>;
        
        if (this.selectedRobot) {
            request = this.missionDatabaseService.getMissionsByRobot(this.selectedRobot);
        } else if (this.selectedMode) {
            request = this.missionDatabaseService.getMissionsByMode(this.selectedMode);
        } else {
            request = this.missionDatabaseService.getAllMissions();
        }

        console.log('Loading missions...');
        request.subscribe({
            next: (missions: Mission[]) => {
                console.log('Missions received:', missions);
                console.log('Number of missions:', missions.length);
                this.missions = missions;
                this.loading = false;
            },
            error: (error: HttpErrorResponse) => {
                console.error('Error loading missions:', error);
                console.error('Error details:', error.error);
                this.error = `Erreur lors du chargement des missions: ${error.message || error.statusText || 'Erreur inconnue'}`;
                this.loading = false;
            }
        });
    }

    loadStats(): void {
        this.missionDatabaseService.getMissionStats().subscribe({
            next: (stats: MissionStats) => {
                this.stats = stats;
            },
            error: (error: HttpErrorResponse) => {
                console.error('Erreur lors du chargement des statistiques:', error);
            }
        });
    }

    filterByRobot(robotName: string | null): void {
        this.selectedRobot = robotName;
        this.selectedMode = null;
        this.loadMissions();
    }

    filterByMode(mode: 'SIMULATION' | 'REAL' | null): void {
        this.selectedMode = mode;
        this.selectedRobot = null;
        this.loadMissions();
    }

    clearFilters(): void {
        this.selectedRobot = null;
        this.selectedMode = null;
        this.loadMissions();
    }

    deleteMission(id: string): void {
        if (confirm('Êtes-vous sûr de vouloir supprimer cette mission?')) {
            this.missionDatabaseService.deleteMission(id).subscribe({
                next: () => {
                    console.log('Mission deleted successfully');
                    this.loadMissions();
                    this.loadStats();
                },
                error: (error: HttpErrorResponse) => {
                    console.error('Error deleting mission:', error);
                    // La suppression a probablement réussi même si le parsing échoue
                    // Recharger quand même les missions
                    this.loadMissions();
                    this.loadStats();
                }
            });
        }
    }

    formatDate(date: Date | string | undefined): string {
        if (!date) return 'N/A';
        const d = typeof date === 'string' ? new Date(date) : date;
        return d.toLocaleString('fr-FR');
    }

    formatDuration(seconds: number): string {
        const hours = Math.floor(seconds / 3600);
        const minutes = Math.floor((seconds % 3600) / 60);
        const secs = seconds % 60;
        
        if (hours > 0) {
            return `${hours}h ${minutes}m ${secs}s`;
        } else if (minutes > 0) {
            return `${minutes}m ${secs}s`;
        } else {
            return `${secs}s`;
        }
    }

    formatDistance(meters: number): string {
        if (meters >= 1000) {
            return `${(meters / 1000).toFixed(2)} km`;
        }
        return `${meters} m`;
    }

    getUniqueRobots(): string[] {
        return [...new Set(this.missions.map(m => m.robotName))];
    }

    populateDatabase(): void {
        if (confirm('Voulez-vous peupler la base de données avec des missions d\'exemple ?\n\nCela va supprimer toutes les missions existantes et créer 6 missions d\'exemple.')) {
            this.loading = true;
            this.error = null;
            
            this.missionDatabaseService.populateDatabase(true).subscribe({
                next: (result) => {
                    console.log('Database populated:', result);
                    this.loadMissions();
                    this.loadStats();
                    this.loading = false;
                },
                error: (error: HttpErrorResponse) => {
                    console.error('Error populating database:', error);
                    this.error = `Erreur lors du peuplement: ${error.message || error.statusText || 'Erreur inconnue'}`;
                    this.loading = false;
                }
            });
        }
    }

    openLogsDialog(mission: Mission): void {
        const logs = this.buildMockLogs(mission);
        this.dialog.open(MissionLogsDialogComponent, {
            width: '760px',
            panelClass: 'mission-logs-dialog-panel',
            data: { mission, logs }
        });
    }

    private buildMockLogs(mission: Mission): MissionLogEntry[] {
        const createdAt = mission.createdAt ? new Date(mission.createdAt) : new Date();
        const phases = ['INIT', 'NAVIGATION', 'ACTION', 'FINALIZE'];
        return [
            {
                timestamp: new Date(createdAt.getTime()),
                level: 'INFO',
                phase: phases[0],
                message: `Mission "${mission.missionName}" initialisée`,
                details: `Robot ${mission.robotName} prêt en mode ${mission.mode}`
            },
            {
                timestamp: new Date(createdAt.getTime() + 1000 * 60),
                level: 'INFO',
                phase: phases[1],
                message: 'Navigation vers le point de départ',
                details: `Distance restante ${(mission.distance / 2).toFixed(1)} m`
            },
            {
                timestamp: new Date(createdAt.getTime() + 1000 * 120),
                level: 'WARN',
                phase: phases[1],
                message: 'Obstacle détecté',
                details: 'Déviation de trajectoire appliquée'
            },
            {
                timestamp: new Date(createdAt.getTime() + 1000 * 180),
                level: 'INFO',
                phase: phases[2],
                message: 'Objectif atteint',
                details: 'Collecte de données en cours'
            },
            {
                timestamp: new Date(createdAt.getTime() + mission.durationSec * 1000),
                level: 'INFO',
                phase: phases[3],
                message: 'Mission terminée',
                details: `Distance parcourue ${mission.distance} m`
            }
        ];
    }
}
