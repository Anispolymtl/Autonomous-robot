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
            this.renderMap();
        });
    }

    resetMap() {
        this.map = undefined;
    }

    renderMap(): void {
        if (!this.map) return;

        const { data, width, height } = this.map;
        const int8View = new Int8Array(data);

        const canvas = document.getElementById('mapCanvas') as HTMLCanvasElement;
        const ctx = canvas.getContext('2d')!;
        const imageData = ctx.createImageData(width, height);

        for (let i = 0; i < int8View.length; i++) {
            const val = int8View[i];
            let color: number;

            if (val === -1) color = 127;
            else color = 255 - (val * 2.55);

            imageData.data[i * 4 + 0] = color;
            imageData.data[i * 4 + 1] = color;
            imageData.data[i * 4 + 2] = color;
            imageData.data[i * 4 + 3] = 255;
        }

        ctx.putImageData(imageData, 0, 0);
        ctx.scale(1, -1);
    }
}
