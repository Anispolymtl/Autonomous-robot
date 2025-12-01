import { Injectable, OnDestroy } from '@angular/core';
import { BehaviorSubject, Observable } from 'rxjs';
import { SocketService } from '@app/services/socket/socket.service';
import { PoseData } from '@app/interfaces/pose-data';


@Injectable({
  providedIn: 'root',
})
export class MissionStateService implements OnDestroy {
  private limo1State$ = new BehaviorSubject<string>('En attente');
  private limo2State$ = new BehaviorSubject<string>('En attente');
  private limo1Position$ = new BehaviorSubject<{ x: number; y: number } | null>(null);
  private limo2Position$ = new BehaviorSubject<{ x: number; y: number } | null>(null);

  constructor(private readonly socketService: SocketService) {}

  connectToSocket() {
    if (!this.socketService.isSocketAlive()) {
      this.socketService.connect('client');
      const socket = this.socketService.getSocket;
      if (socket) {
        socket.once('connect', () => {
          console.log('[STATE] Socket connected');
          this.configureStateSocketFeatures();
        });
      }
    } else {
      this.configureStateSocketFeatures();
    }
  }

  private configureStateSocketFeatures() {
    this.socketService.on('stateUpdate', (payload: { robot: string; state: string }) => {
      if (!payload || !payload.robot || !payload.state) {
        console.log('[STATE] Abort message');
        return;
      }

      console.log(`[CLIENT] État reçu → ${payload.robot}: ${payload.state}`);

      if (payload.robot === 'limo1') {
        this.limo1State$.next(payload.state);
      } else if (payload.robot === 'limo2') {
        this.limo2State$.next(payload.state);
      }
    });

    this.socketService.on('poseUpdate', (payload: { robot: string; poseData: PoseData }) => {
      if (!payload?.robot || !payload.poseData?.pose?.position) return;
      const { x, y } = payload.poseData.pose.position;
      const position = { x, y };

      if (payload.robot === 'limo1') {
        this.limo1Position$.next(position);
      } else if (payload.robot === 'limo2') {
        this.limo2Position$.next(position);
      }
    });
  }

  getLimo1State$(): Observable<string> {
    return this.limo1State$.asObservable();
  }

  getLimo2State$(): Observable<string> {
    return this.limo2State$.asObservable();
  }

  getLimo1Position$(): Observable<{ x: number; y: number } | null> {
    return this.limo1Position$.asObservable();
  }

  getLimo2Position$(): Observable<{ x: number; y: number } | null> {
    return this.limo2Position$.asObservable();
  }

  disconnect() {
    this.socketService.disconnect();
  }

  ngOnDestroy() {
    this.disconnect();
  }
}
