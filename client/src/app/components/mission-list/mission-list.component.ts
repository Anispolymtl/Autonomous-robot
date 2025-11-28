import { Component, OnInit } from '@angular/core';
import { CommonModule } from '@angular/common';
import { FormsModule } from '@angular/forms';
import { Mission, MissionLogEntry, MissionStats } from '@app/interfaces/mission';
import { HttpErrorResponse } from '@angular/common/http';
import { Observable } from 'rxjs';
import { MissionDatabaseService } from '@app/services/mission-database/mission-database.service';
import { MatDialog, MatDialogModule } from '@angular/material/dialog';
import { MissionLogsDialogComponent } from '@app/components/mission-logs-dialog/mission-logs-dialog.component';
import { MissionMapsDialogComponent } from '@app/components/mission-maps-dialog/mission-maps-dialog.component';

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
    loadingMore = false;
    allLoaded = false;
    error: string | null = null;
    mapsLoadingId: string | null = null;
    selectedMode: 'SIMULATION' | 'REAL' | null = null;
    readonly pageSize = 6;

    constructor(
        private missionDatabaseService: MissionDatabaseService,
        private dialog: MatDialog
    ) { }

    ngOnInit(): void {
        this.loadMissions(true);
        this.loadStats();
    }

    loadMissions(reset = false): void {
        if (reset) {
            this.missions = [];
            this.allLoaded = false;
        }

        const skip = this.missions.length;
        const isInitialLoad = reset || skip === 0;
        this.loading = isInitialLoad;
        this.loadingMore = !isInitialLoad;
        this.error = null;
        
        let request: Observable<Mission[]>;
        
        if (this.selectedMode) {
            request = this.missionDatabaseService.getMissionsByMode(this.selectedMode, this.pageSize, skip);
        } else {
            request = this.missionDatabaseService.getAllMissions(this.pageSize, skip);
        }

        console.log('Loading missions...');
        request.subscribe({
            next: (missions: Mission[]) => {
                console.log('Missions received:', missions);
                console.log('Number of missions:', missions.length);
                this.missions = reset ? missions : [...this.missions, ...missions];
                this.updateLoadedState(missions.length);
            },
            error: (error: HttpErrorResponse) => {
                console.error('Error loading missions:', error);
                console.error('Error details:', error.error);
                this.error = `Erreur lors du chargement des missions: ${error.message || error.statusText || 'Erreur inconnue'}`;
                this.loading = false;
                this.loadingMore = false;
            },
            complete: () => {
                this.loading = false;
                this.loadingMore = false;
            }
        });
    }

    loadStats(): void {
        this.missionDatabaseService.getMissionStats().subscribe({
            next: (stats: MissionStats) => {
                this.stats = stats;
                this.syncLoadedStateWithStats();
            },
            error: (error: HttpErrorResponse) => {
                console.error('Erreur lors du chargement des statistiques:', error);
            }
        });
    }

    filterByMode(mode: 'SIMULATION' | 'REAL' | null): void {
        this.selectedMode = mode;
        this.loadMissions(true);
    }

    clearFilters(): void {
        this.selectedMode = null;
        this.loadMissions(true);
    }

    deleteMission(id: string): void {
        if (confirm('Êtes-vous sûr de vouloir supprimer cette mission?')) {
            this.missionDatabaseService.deleteMission(id).subscribe({
                next: () => {
                    console.log('Mission deleted successfully');
                    this.loadMissions(true);
                    this.loadStats();
                },
                error: (error: HttpErrorResponse) => {
                    console.error('Error deleting mission:', error);
                    // La suppression a probablement réussi même si le parsing échoue
                    // Recharger quand même les missions
                    this.loadMissions(true);
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

    formatRobots(robots: string[] | undefined): string {
        if (!robots || robots.length === 0) {
            return 'N/A';
        }
        return robots.join(' & ');
    }

    openLogsDialog(mission: Mission): void {
        const logs = this.extractLogsFromMission(mission);
        this.dialog.open(MissionLogsDialogComponent, {
            width: '760px',
            panelClass: 'mission-logs-dialog-panel',
            data: { mission, logs }
        });
    }

    openMapsDialog(mission: Mission): void {
        if (!mission._id) {
            console.warn('Impossible de charger les cartes: identifiant de mission manquant.');
            return;
        }

        this.mapsLoadingId = mission._id;

        this.missionDatabaseService.getMissionById(mission._id).subscribe({
            next: (missionWithMaps: Mission) => {
                if (!missionWithMaps.maps) {
                    alert('Aucune carte disponible pour cette mission.');
                    return;
                }

                this.dialog.open(MissionMapsDialogComponent, {
                    width: '760px',
                    panelClass: 'mission-logs-dialog-panel',
                    data: { mission: missionWithMaps }
                });
            },
            error: (error: HttpErrorResponse) => {
                console.error('Error loading mission maps:', error);
                alert('Erreur lors du chargement des cartes.');
                this.mapsLoadingId = null;
            },
            complete: () => {
                this.mapsLoadingId = null;
            }
        });
    }

    loadMore(): void {
        if (this.loading || this.loadingMore || this.allLoaded) return;
        this.loadMissions();
    }

    private extractLogsFromMission(mission: Mission): MissionLogEntry[] {
        const candidateKeys = ['logs', 'logEntries', 'events', 'history', 'log', 'logHistory', 'telemetry', 'missionLogs'];
        const missionAsRecord = mission as unknown as Record<string, unknown>;

        for (const key of candidateKeys) {
            const value = missionAsRecord[key];
            if (Array.isArray(value)) {
                return value as MissionLogEntry[];
            }
        }

        if (Array.isArray(mission.logs)) {
            return mission.logs;
        }

        return [];
    }

    private updateLoadedState(lastBatchSize: number): void {
        const total = this.stats?.total;
        const canUseStats = !this.selectedMode && typeof total === 'number';
        if (canUseStats && this.missions.length >= total) {
            this.allLoaded = true;
            return;
        }
        this.allLoaded = lastBatchSize < this.pageSize;
    }

    private syncLoadedStateWithStats(): void {
        const total = this.stats?.total;
        if (this.selectedMode || typeof total !== 'number') return;
        this.allLoaded = this.missions.length >= total;
    }
}
