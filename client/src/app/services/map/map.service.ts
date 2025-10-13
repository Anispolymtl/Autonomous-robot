import { Injectable } from '@angular/core';
import { HttpClient } from '@angular/common/http';
import { Observable } from 'rxjs';
import { environment } from 'src/environments/environment';

@Injectable({
    providedIn: 'root'
})
export class MapService {
    constructor(private http: HttpClient) { }

    getMap(): Observable<Blob> {
      return this.http.get(`${environment.serverUrl}/static/map/map.png`, { responseType: 'blob' });
    }
}