import { Injectable } from '@angular/core';
import { HttpClient, HttpParams } from '@angular/common/http';
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
    getAllMissions(limit?: number, skip?: number): Observable<Mission[]> {
        const params = this.buildPaginationParams(limit, skip);
        return this.http.get<Mission[]>(`${this.missionsUrl}/`, { params });
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
    getMissionsByRobot(robotName: string, limit?: number, skip?: number): Observable<Mission[]> {
        const params = this.buildPaginationParams(limit, skip);
        return this.http.get<Mission[]>(`${this.missionsUrl}/robot/${robotName}`, { params });
    }

    /**
     * Récupère toutes les missions d'un mode spécifique (SIMULATION ou REAL)
     */
    getMissionsByMode(mode: 'SIMULATION' | 'REAL', limit?: number, skip?: number): Observable<Mission[]> {
        const params = this.buildPaginationParams(limit, skip);
        return this.http.get<Mission[]>(`${this.missionsUrl}/mode/${mode}`, { params });
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

    private buildPaginationParams(limit?: number, skip?: number): HttpParams {
        let params = new HttpParams();
        if (limit !== undefined) params = params.set('limit', limit.toString());
        if (skip !== undefined) params = params.set('skip', skip.toString());
        return params;
    }

}
