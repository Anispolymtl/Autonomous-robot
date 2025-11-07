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

interface PoseData {
    header: { frame_id: string; stamp?: { sec: number; nanosec: number } };
    pose: {
        position: { x: number; y: number; z: number };
        orientation: { x: number; y: number; z: number; w: number };
    };
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
    selectedPoint: MapCoordinate | undefined;
    selectedCanvasCoord: { x: number; y: number } | undefined;
    pointList: MapCoordinate[] = [];
    private pointCanvasCoords: { x: number; y: number }[] = [];
    private robotPoses: Record<string, PoseData | undefined> = {};

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
        this.socketService.on(MapEvent.PoseUpdate, (payload: { robot: string; poseData: PoseData }) => {
            if (!payload?.robot || !payload.poseData || payload.robot == 'limo2') return; //A enlever le check pour si limo2
            this.robotPoses[payload.robot] = payload.poseData;
            this.renderMap();
        });
    }

    resetMap() {
        this.map = undefined;
        this.originCanvasPosition = undefined;
        this.robotPoses = {};
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
            canvas.style.width = `${width}px`;
            canvas.style.height = `${height}px`;
        }

        const ctx = canvas.getContext('2d');
        if (!ctx) return;
        ctx.imageSmoothingEnabled = false;

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
        this.drawMarkers(ctx);
        this.drawRobotPoses(ctx);
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
    
    onCanvasClick(event: MouseEvent): void {
        const canvas = document.getElementById('mapCanvas') as HTMLCanvasElement | null;
        if (!canvas) return;

        const rect = canvas.getBoundingClientRect();
        if (!rect.width || !rect.height) return;

        const scaleX = canvas.width / rect.width;
        const scaleY = canvas.height / rect.height;
        const canvasX = (event.clientX - rect.left) * scaleX;
        const canvasY = (event.clientY - rect.top) * scaleY;

        const coordinate = this.canvasPointToMapCoordinate(canvasX, canvasY);
        if (coordinate) {
            this.selectedPoint = coordinate;
            this.selectedCanvasCoord = this.snapToPixel(canvasX, canvasY);
            this.renderMap();
            console.log('Selected map coordinate:', coordinate);
        }
    }

    addPoint(): void{
        if(!this.selectedPoint || !this.selectedCanvasCoord) return;
        this.pointList.push(this.selectedPoint);
        this.pointCanvasCoords.push(this.selectedCanvasCoord);
        this.selectedPoint = undefined;
        this.selectedCanvasCoord = undefined;
        this.renderMap();
    }

    removePoint(index: number): void {
        if (index < 0 || index >= this.pointList.length) return;
        this.pointList.splice(index, 1);
        this.pointCanvasCoords.splice(index, 1);
        this.renderMap();
    }

    sendCoords(): void {
        if (!this.pointList.length) {
            console.log('No coordinates to send.');
            return;
        }
        console.log('Sending coordinates:', this.pointList);
        this.socketService.send('pointlist', {robot: 'limo1', points: this.pointList.map(point => point.world)});
        this.pointList = [];
        this.pointCanvasCoords = [];
        this.renderMap();
    }
    
    private drawOriginMarker(ctx: CanvasRenderingContext2D): void {
        if (!this.originCanvasPosition) return;
        ctx.save();
        ctx.fillStyle = '#e53935';
        const size = 6;
        const x = Math.max(size / 2, Math.round(this.originCanvasPosition.x));
        const y = Math.min(ctx.canvas.height - size / 2, Math.round(this.originCanvasPosition.y));
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

    private drawMarkers(ctx: CanvasRenderingContext2D): void {
        ctx.imageSmoothingEnabled = false;
        const savedPointColor = '#1e88e5';
        const selectedPointColor = '#ff1744';
        this.drawPathThroughPoints(ctx, this.pointCanvasCoords, savedPointColor);
        this.pointCanvasCoords.forEach(({ x, y }) => this.drawPointMarker(ctx, x, y, savedPointColor));
        if (this.selectedCanvasCoord) {
            this.drawPointMarker(ctx, this.selectedCanvasCoord.x, this.selectedCanvasCoord.y, selectedPointColor);
        }
    }


    private drawPointMarker(ctx: CanvasRenderingContext2D, x: number, y: number, color: string): void {
        ctx.save();
        ctx.fillStyle = color;
        const size = 3;
        const px = Math.round(x);
        const py = Math.round(y);
        ctx.fillRect(px - size / 2, py - size / 2, size, size);
        ctx.restore();
    }

    private drawPathThroughPoints(
        ctx: CanvasRenderingContext2D,
        points: { x: number; y: number }[],
        color: string,
    ): void {
        if (points.length < 2) return;
        ctx.save();
        ctx.imageSmoothingEnabled = false;
        ctx.strokeStyle = color;
        ctx.lineWidth = 1;
        ctx.lineCap = 'round';
        ctx.lineJoin = 'round';
        ctx.beginPath();
        ctx.moveTo(Math.round(points[0].x) + 0.5, Math.round(points[0].y) + 0.5);
        for (let i = 1; i < points.length; i++) {
            ctx.lineTo(Math.round(points[i].x) + 0.5, Math.round(points[i].y) + 0.5);
        }
        ctx.stroke();
        ctx.restore();
    }

    private snapToPixel(x: number, y: number): { x: number; y: number } {
        return { x: Math.round(x), y: Math.round(y) };
    }

    private worldToCanvas(worldX: number, worldY: number): { x: number; y: number } | undefined {
        if (!this.map) return undefined;
        const { origin, resolution, width, height } = this.map;
        if (!resolution || !width || !height) return undefined;

        const dx = worldX - origin.position.x;
        const dy = worldY - origin.position.y;

        const offsetX = dx * this.yawCos + dy * this.yawSin;
        const offsetY = -dx * this.yawSin + dy * this.yawCos;

        const gridX = offsetX / resolution - 0.5;
        const gridY = offsetY / resolution - 0.5;

        if (gridX < 0 || gridX >= width || gridY < 0 || gridY >= height) return undefined;

        return { x: gridX, y: height - 1 - gridY };
    }

    private quaternionToYaw(quaternion: { x: number; y: number; z: number; w: number }): number {
        const { w, x, y, z } = quaternion;
        return Math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z));
    }

    private orientationToCanvasVector(yaw: number): { x: number; y: number } {
        const dirWorldX = Math.cos(yaw);
        const dirWorldY = Math.sin(yaw);
        const dirGridX = dirWorldX * this.yawCos + dirWorldY * this.yawSin;
        const dirGridY = -dirWorldX * this.yawSin + dirWorldY * this.yawCos;
        return { x: dirGridX, y: -dirGridY };
    }

    private drawRobotPoses(ctx: CanvasRenderingContext2D): void {
        Object.values(this.robotPoses).forEach((poseData) => {
            if (!poseData) return;
            const canvasPos = this.worldToCanvas(poseData.pose.position.x, poseData.pose.position.y);
            if (!canvasPos) return;
            const yaw = this.quaternionToYaw(poseData.pose.orientation);
            const direction = this.orientationToCanvasVector(yaw);
            this.drawRobotTriangle(ctx, canvasPos, direction);
        });
    }

    private drawRobotTriangle(
        ctx: CanvasRenderingContext2D,
        center: { x: number; y: number },
        direction: { x: number; y: number },
    ): void {
        const length = 14;
        const baseWidth = 9;

        const magnitude = Math.hypot(direction.x, direction.y);
        if (!magnitude) return;
        const dirX = direction.x / magnitude;
        const dirY = direction.y / magnitude;

        const perpX = -dirY;
        const perpY = dirX;
        const halfBase = baseWidth / 2;
        const backOffset = length * 0.4;

        const tip = { x: center.x + dirX * length, y: center.y + dirY * length };
        const rearCenter = { x: center.x - dirX * backOffset, y: center.y - dirY * backOffset };
        const left = { x: rearCenter.x + perpX * halfBase, y: rearCenter.y + perpY * halfBase };
        const right = { x: rearCenter.x - perpX * halfBase, y: rearCenter.y - perpY * halfBase };

        ctx.save();
        ctx.fillStyle = '#00FF00';
        ctx.strokeStyle = '#006600';
        ctx.lineWidth = 1;
        ctx.beginPath();
        ctx.moveTo(tip.x, tip.y);
        ctx.lineTo(left.x, left.y);
        ctx.lineTo(right.x, right.y);
        ctx.closePath();
        ctx.fill();
        ctx.stroke();
        ctx.restore();
    }
}
