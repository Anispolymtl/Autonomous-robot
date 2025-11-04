import { Injectable, OnInit, signal, WritableSignal } from '@angular/core';
import { SocketService } from '@app/services/socket.service';
import { MapEvent } from '@common/enums/sockets-events'
import { Map } from '@common/interfaces/map'

@Injectable({
    providedIn: 'root',
})
export class MapService implements OnInit {
    map: WritableSignal<Map[]> = signal([]);

    constructor(private readonly socketService: SocketService) {}

    ngOnInit(): void {
        this.connectToSocket()
    }
    
    get isSocketAlive(): boolean {
        return this.socketService.isSocketAlive();
    }

    connectToSocket() {
        if (!this.socketService.isSocketAlive()) {
            this.socketService.connect();
            this.configureMapSocketFeatures();
        }
    }

    configureMapSocketFeatures() {
        this.socketService.on(MapEvent.RecoverMap, (recoveredMap: Map[]) => {
            this.map.set([...recoveredMap]);
            console.log(this.map)
        });
    }

    resetMap() {
        this.map.set([]);
    }
}
