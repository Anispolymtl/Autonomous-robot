import { Injectable, OnDestroy } from '@angular/core';
import { BehaviorSubject, Observable } from 'rxjs';
import { SocketService } from '@app/services/socket.service';


@Injectable({
  providedIn: 'root',
})
export class MissionStateService implements OnDestroy {
  private limo1State$ = new BehaviorSubject<string>('En attente');
  private limo2State$ = new BehaviorSubject<string>('En attente');

  constructor(private readonly socketService: SocketService) {}

  /** Connexion aux namespaces Socket.IO (un pour chaque robot) */
    connectToSocket() {
        if (!this.socketService.isSocketAlive()) {
            this.socketService.connect('client');
            this.configureStateSocketFeatures();
        }
    }
    configureStateSocketFeatures() {
        // ✅ On écoute les mises à jour globales du serveur
        this.socketService.on('stateUpdate', (payload: {robot: string, state: string}) => {
            if (!payload || !payload.robot || !payload.state){
                console.log('[STATE] Abort  message')
                return;
            }

        console.log(`[CLIENT] État reçu → ${payload.robot}: ${payload.state}`);

        if (payload.robot === 'limo1') {
            this.limo1State$.next(payload.state);
        } else if (payload.robot === 'limo2') {
            this.limo2State$.next(payload.state);
        }
        });
  }

  /** Observable du state pour Limo 1 */
  getLimo1State$(): Observable<string> {
    return this.limo1State$.asObservable();
  }

  /** Observable du state pour Limo 2 */
  getLimo2State$(): Observable<string> {
    return this.limo2State$.asObservable();
  }

  /** Déconnexion socket */
  disconnect() {
    this.socketService.disconnect();
  }

  ngOnDestroy() {
    this.disconnect();
  }
}
