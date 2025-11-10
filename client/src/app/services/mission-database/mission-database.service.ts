import { Injectable } from '@angular/core';
import { HttpClient } from '@angular/common/http';
import { Observable } from 'rxjs';
import { environment } from 'src/environments/environment';
import { Mission, CreateMissionDto, UpdateMissionDto, MissionStats } from '@app/interfaces/mission';

@Injectable({
    providedIn: 'root'
})
export class MissionDatabaseService {
    private missionsUrl = `${environment.serverUrl}/api/missions`;

    constructor(private http: HttpClient) { }

    /**
     * Récupère toutes les missions de la base de données
     */
    getAllMissions(): Observable<Mission[]> {
        return this.http.get<Mission[]>(`${this.missionsUrl}/`);
    }

    /**
     * Récupère une mission par son ID
     */
    getMissionById(id: string): Observable<Mission> {
        return this.http.get<Mission>(`${this.missionsUrl}/${id}`);
    }

    /**
     * Récupère toutes les missions d'un robot spécifique
     */
    getMissionsByRobot(robotName: string): Observable<Mission[]> {
        return this.http.get<Mission[]>(`${this.missionsUrl}/robot/${robotName}`);
    }

    /**
     * Récupère toutes les missions d'un mode spécifique (SIMULATION ou REAL)
     */
    getMissionsByMode(mode: 'SIMULATION' | 'REAL'): Observable<Mission[]> {
        return this.http.get<Mission[]>(`${this.missionsUrl}/mode/${mode}`);
    }

    /**
     * Récupère les statistiques des missions
     */
    getMissionStats(): Observable<MissionStats> {
        return this.http.get<MissionStats>(`${this.missionsUrl}/stats/overview`);
    }

    /**
     * Crée une nouvelle mission
     */
    createMission(mission: CreateMissionDto): Observable<Mission> {
        return this.http.post<Mission>(`${this.missionsUrl}/`, mission);
    }

    /**
     * Met à jour une mission existante
     */
    updateMission(mission: UpdateMissionDto): Observable<Mission> {
        return this.http.patch<Mission>(`${this.missionsUrl}/`, mission);
    }

    /**
     * Supprime une mission par son ID
     */
    deleteMission(id: string): Observable<{ message: string }> {
        return this.http.delete<{ message: string }>(`${this.missionsUrl}/${id}`);
    }

    /**
     * Peuple la base de données avec des missions d'exemple
     * @param force Si true, supprime toutes les missions existantes avant de peupler
     */
    populateDatabase(force: boolean = false): Observable<{ created: number; message: string }> {
        return this.http.post<{ created: number; message: string }>(`${this.missionsUrl}/populate`, { force });
    }
}
