import { Injectable } from '@angular/core';
import { HttpClient } from '@angular/common/http';
import { Observable } from 'rxjs';
import { environment } from 'src/environments/environment';

@Injectable({
    providedIn: 'root'
})
export class IdentifyService {
    private apiUrl = `${environment.serverUrl}/api/identify`;

    constructor(private http: HttpClient) { }

    identifyRobot(robotId: number): Observable<any> {
        return this.http.get<any>(`${this.apiUrl}`, { params: { id: robotId } });
    }
}