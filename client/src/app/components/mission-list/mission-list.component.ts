import { Component, OnInit } from '@angular/core';
import { CommonModule } from '@angular/common';
import { FormsModule } from '@angular/forms';
import { MissionDatabaseService } from '@app/services/mission-database/mission-database.service';
import { Mission, MissionStats } from '@app/interfaces/mission';
import { HttpErrorResponse } from '@angular/common/http';
import { Observable } from 'rxjs';

@Component({
    selector: 'app-mission-list',
    standalone: true,
    imports: [CommonModule, FormsModule],
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

    constructor(private missionDatabaseService: MissionDatabaseService) { }

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
}

