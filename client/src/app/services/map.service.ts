import { Injectable } from '@angular/core';
import { SocketService } from '@app/services/socket.service';
import { MapEvent } from '@common/enums/sockets-events';

interface MapOrientation {
    position: { x: number; y: number; z: number };
    orientation: { x: number; y: number; z: number; w: number };
}

interface OccupancyGrid {
    data: Int8Array;
    height: number;
    width: number;
    resolution: number;
    origin: MapOrientation;
}

export interface MapCoordinate {
    cell: { x: number; y: number };
    world: { x: number; y: number };
}

@Injectable({
    providedIn: 'root',
})
export class MapService {

    map: OccupancyGrid | undefined;
    originCanvasPosition: { x: number; y: number } | undefined;
    private yaw = 0;
    private yawCos = 1;
    private yawSin = 0;

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
            this.map = {
                data: this.normaliseMapData(recoveredMap?.data),
                height: recoveredMap?.info?.height ?? 0,
                width: recoveredMap?.info?.width ?? 0,
                resolution: recoveredMap?.info?.resolution ?? 1,
                origin: recoveredMap?.info?.origin ?? {
                    position: { x: 0, y: 0, z: 0 },
                    orientation: { x: 0, y: 0, z: 0, w: 1 },
                },
            };
            this.updateOrientationCache();
            this.renderMap();
        });
    }

    resetMap() {
        this.map = undefined;
        this.originCanvasPosition = undefined;
    }

    renderMap(): void {
        if (!this.map) return;

        const { data, width, height } = this.map;
        if (!data.length || !width || !height) return;

        const canvas = document.getElementById('mapCanvas') as HTMLCanvasElement | null;
        if (!canvas) return;

        if (canvas.width !== width || canvas.height !== height) {
            canvas.width = width;
            canvas.height = height;
        }

        const ctx = canvas.getContext('2d');
        if (!ctx) return;

        const imageData = ctx.createImageData(width, height);
        const pixelData = imageData.data;

        for (let i = 0; i < data.length; i++) {
            const value = data[i];
            const color = value === -1 ? 127 : Math.max(0, Math.min(255, 255 - value * 2.55));

            const gridX = i % width;
            const gridY = Math.floor(i / width);
            const canvasX = gridX;
            const canvasY = height - 1 - gridY;
            const destIndex = (canvasY * width + canvasX) * 4;

            pixelData[destIndex] = color;
            pixelData[destIndex + 1] = color;
            pixelData[destIndex + 2] = color;
            pixelData[destIndex + 3] = 255;
        }

        ctx.putImageData(imageData, 0, 0);
        this.originCanvasPosition = { x: 0.5, y: height - 0.5 };
        this.drawOriginMarker(ctx);
    }

    getOriginInWorld(): { x: number; y: number } | undefined {
        if (!this.map) return undefined;
        const { position } = this.map.origin;
        return { x: position.x, y: position.y };
    }

    canvasPointToMapCoordinate(canvasX: number, canvasY: number): MapCoordinate | undefined {
        if (!this.map) return undefined;

        const { width, height } = this.map;
        if (canvasX < 0 || canvasX >= width || canvasY < 0 || canvasY >= height) return undefined;

        const canvasXInt = Math.floor(canvasX);
        const canvasYInt = Math.floor(canvasY);
        const gridX = canvasXInt;
        const gridY = height - 1 - canvasYInt;

        return {
            cell: { x: gridX, y: gridY },
            world: this.gridToWorld(gridX, gridY),
        };
    }

    private drawOriginMarker(ctx: CanvasRenderingContext2D): void {
        if (!this.originCanvasPosition) return;
        ctx.save();
        ctx.fillStyle = '#e53935';
        const size = 6;
        const x = Math.max(size / 2, this.originCanvasPosition.x);
        const y = Math.min(ctx.canvas.height - size / 2, this.originCanvasPosition.y);
        ctx.fillRect(x - size / 2, y - size / 2, size, size);
        ctx.restore();
    }

    private gridToWorld(gridX: number, gridY: number): { x: number; y: number } {
        if (!this.map) return { x: 0, y: 0 };
        const { resolution, origin } = this.map;
        const offsetX = (gridX + 0.5) * resolution;
        const offsetY = (gridY + 0.5) * resolution;

        const x = origin.position.x + offsetX * this.yawCos - offsetY * this.yawSin;
        const y = origin.position.y + offsetX * this.yawSin + offsetY * this.yawCos;

        return { x, y };
    }

    private normaliseMapData(rawData: unknown): Int8Array {
        if (rawData && typeof rawData === 'object' && 'data' in (rawData as { data?: unknown })) {
            return this.normaliseMapData((rawData as { data: unknown }).data);
        }
        if (rawData instanceof Int8Array) return rawData;
        if (rawData instanceof ArrayBuffer) return new Int8Array(rawData);
        if (ArrayBuffer.isView(rawData)) {
            const view = rawData as ArrayBufferView;
            return new Int8Array(view.buffer.slice(view.byteOffset, view.byteOffset + view.byteLength));
        }
        if (Array.isArray(rawData)) return Int8Array.from(rawData);
        console.warn('Unsupported map data format received from socket');
        return new Int8Array();
    }

    private updateOrientationCache(): void {
        if (!this.map) {
            this.yaw = 0;
            this.yawCos = 1;
            this.yawSin = 0;
            return;
        }
        const { w, x, y, z } = this.map.origin.orientation;
        // Extract yaw around Z axis from quaternion.
        this.yaw = Math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z));
        this.yawCos = Math.cos(this.yaw);
        this.yawSin = Math.sin(this.yaw);
    }
}
