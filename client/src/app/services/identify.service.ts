import { Injectable } from '@angular/core';
import { HttpClient } from '@angular/common/http';
import { Observable } from 'rxjs';

@Injectable({
    providedIn: 'root'
})
export class IdentifyService {
    private apiUrl = 'http://localhost:3000/identify';

    constructor(private http: HttpClient) { }

    identifyRobot(): Observable<any> {
        return this.http.get<any>(this.apiUrl);
    }
}