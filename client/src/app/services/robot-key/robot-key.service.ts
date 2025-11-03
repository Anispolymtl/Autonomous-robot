import { Injectable } from '@angular/core';
import { HttpClient } from '@angular/common/http';
import { delay, map, of } from 'rxjs';

@Injectable({ providedIn: 'root' })
export class RobotKeyService {
  // Passez à true si vous avez une API à appeler.
  private readonly USE_HTTP = false;

  constructor(private http: HttpClient) {}

  /** Clé simulée: `${robotName}-${teamName}-${ISODate}` */
  getKey(robotName: string, teamName: string) {
    if (this.USE_HTTP) {
      // Adaptez l’URL/payload à votre backend.
      return this.http
        .post<{ key: string }>('/api/robot-key', { robotName, teamName })
        .pipe(map(res => res.key));
    }
    const iso = new Date().toISOString();
    const key = `${robotName}-${teamName}-${iso}`;
    return of(key).pipe(delay(500));
  }
}
