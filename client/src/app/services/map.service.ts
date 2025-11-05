import { Injectable } from '@angular/core';
import { SocketService } from '@app/services/socket.service';
import { MapEvent } from '@common/enums/sockets-events'

@Injectable({
    providedIn: 'root',
})
export class MapService {

    map: {data: ArrayBuffer, height: number, width: number} | undefined;

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
            this.map = {data: recoveredMap.data, height: recoveredMap.info.height, width: recoveredMap.info.width}
            console.log(recoveredMap);
            console.log(this.map);
        });
    }

    resetMap() {
        this.map = undefined;
    }
}
