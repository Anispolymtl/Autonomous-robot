import { Injectable, signal, WritableSignal } from '@angular/core';
import { SocketService } from '@app/services/socket.service';
import { MapEvent } from '@common/enums/sockets-events'

@Injectable({
    providedIn: 'root',
})
export class MapService {
    map: WritableSignal<any> = signal([]);

    constructor(private readonly socketService: SocketService) {}
    
    get isSocketAlive(): boolean {
        return this.socketService.isSocketAlive();
    }

    connectToSocket() {
        if (!this.socketService.isSocketAlive()) {
            this.socketService.connect('client');
            this.configureMapSocketFeatures();
        }
    }

    configureMapSocketFeatures() {
        this.socketService.on(MapEvent.RecoverMap, (recoveredMap: any) => {
            this.map.set(recoveredMap);
            console.log(recoveredMap)
        });
    }

    resetMap() {
        this.map.set([]);
    }
}
