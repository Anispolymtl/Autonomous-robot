import { Injectable } from '@angular/core';
import { HttpClient } from '@angular/common/http';
import { Observable } from 'rxjs';
import { environment } from 'src/environments/environment';

@Injectable({
    providedIn: 'root'
})
export class MissionService {
    private missionUrl = `${environment.serverUrl}/api/mission`;

    constructor(private http: HttpClient) { }

    startMission(): Observable<any> {
        return this.http.get<any>(`${this.missionUrl}/start`);
    }

    cancelMission(): Observable<any> {
        return this.http.get<any>(`${this.missionUrl}/stop`);
    }
}