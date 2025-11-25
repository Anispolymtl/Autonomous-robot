import { Injectable } from '@angular/core';
import { MapObject } from '@app/components/map/map.component';
import { MapCoordinate } from '@app/interfaces/map-coordinate';
import { OccupancyGrid } from '@common/interfaces/occupancy-grid';
import { Orientation } from '@app/interfaces/orientation';
import { SocketService } from '@app/services/socket/socket.service';
import { MissionSessionService } from '@app/services/mission-session/mission-session.service';
import { MissionLogDetails } from '@common/interfaces/mission-log-entry';

@Injectable({
    providedIn: 'root',
})
export class MapService {
    constructor(
        private readonly socketService: SocketService,
        private readonly missionSessionService: MissionSessionService
    ) {
    }

    renderMap(canvas: HTMLCanvasElement, mapObj: MapObject): void {
        if (!mapObj.map) return;
        console.log('render')

        const { data, width, height } = mapObj.map;
        if (!data.length || !width || !height) return;
        const ctx = canvas.getContext('2d');
        if (!ctx) return;
        this.generateBareMap(canvas, mapObj.map, ctx)
        this.drawMarkers(ctx, mapObj);
        this.drawRobotPoses(ctx, mapObj);
    }

    generateBareMap(canvas: HTMLCanvasElement, map: OccupancyGrid, ctx: CanvasRenderingContext2D) {
        // Logique separee pour laffichage des maps de DB
        const { data, width, height } = map;
        if (!data.length || !width || !height) return;

        if (canvas.width !== width || canvas.height !== height) {
            canvas.width = width;
            canvas.height = height;
        }

        canvas.style.width = '100%';
        canvas.style.height = 'auto';

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
    }

    getOriginInWorld(map: OccupancyGrid): { x: number; y: number } | undefined {
        if (!map) return undefined;
        const { position } = map.origin;
        return { x: position.x, y: position.y };
    }

    canvasPointToMapCoordinate(mapObj: MapObject, canvasX: number, canvasY: number): MapCoordinate | undefined {
        if (!mapObj.map) return undefined;
        
        const { width, height } = mapObj.map;
        if (canvasX < 0 || canvasX >= width || canvasY < 0 || canvasY >= height) return undefined;
        
        const canvasXInt = Math.floor(canvasX);
        const canvasYInt = Math.floor(canvasY);
        const gridX = canvasXInt;
        const gridY = height - 1 - canvasYInt;
        
        return {
            cell: { x: gridX, y: gridY },
            world: this.gridToWorld(gridX, gridY, mapObj),
        };
    }
    
    worldPointToMapCoordinate(mapObj: MapObject, worldX: number, worldY: number): MapCoordinate | undefined {
        if (!mapObj.map) return undefined;
        const { width, height } = mapObj.map;
        const canvasCoord = this.worldToCanvas(worldX, worldY, mapObj);
        if (!canvasCoord) return undefined;

        const gridX = Math.floor(canvasCoord.x);
        const gridY = height - 1 - Math.floor(canvasCoord.y);
        if (gridX < 0 || gridX >= width || gridY < 0 || gridY >= height) return undefined;

        return {
            cell: { x: gridX, y: gridY },
            world: { x: worldX, y: worldY },
        };
    }
    
    onCanvasClick(event: MouseEvent, mapObj: MapObject, canvas: HTMLCanvasElement): void {
        const rect = canvas.getBoundingClientRect();
        if (!rect.width || !rect.height) return;

        const scaleX = canvas.width / rect.width;
        const scaleY = canvas.height / rect.height;
        const canvasX = (event.clientX - rect.left) * scaleX;
        const canvasY = (event.clientY - rect.top) * scaleY;

        const coordinate = this.canvasPointToMapCoordinate(mapObj, canvasX, canvasY);
        if (coordinate) {
            mapObj.selectedPoint = coordinate;
            mapObj.selectedCanvasCoord = this.snapToPixel(canvasX, canvasY);
            this.renderMap(canvas, mapObj);
            console.log('Selected map coordinate:', coordinate);
        }
    }

    sendPoint(mapObj: MapObject): void{
        const point = mapObj.selectedPoint;
        if(!point || !mapObj.selectedCanvasCoord) return;
        this.socketService.send('point', {robot: mapObj.frame, point: point.world});
        this.logCommand(mapObj.frame, 'waypoint_added', {
            worldX: Number(point.world.x.toFixed(3)),
            worldY: Number(point.world.y.toFixed(3)),
            cellX: point.cell.x,
            cellY: point.cell.y,
        });
        mapObj.selectedPoint = undefined;
        mapObj.selectedCanvasCoord = undefined;
    }

    removePoint(index: number, mapObj: MapObject): void {
        if (index < 0 || index >= mapObj.pointList.length) return;
        const removedPoint = mapObj.pointList[index];
        this.socketService.send('removePoint', {robot: mapObj.frame, index});
        this.logCommand(mapObj.frame, 'waypoint_removed', {
            index,
            worldX: removedPoint?.world?.x ?? null,
            worldY: removedPoint?.world?.y ?? null,
        });
    }

    sendGoal(mapObj: MapObject): void {
        if (!mapObj.pointList.length) {
            console.log('No coordinates to send.');
            return;
        }
        console.log('Sending objective:', mapObj.pointList);
        this.socketService.send('startNavGoal', {robot: mapObj.frame});
        const serializedWaypoints = mapObj.pointList
            .map((point, idx) => `#${idx + 1}(${point.world.x.toFixed(3)},${point.world.y.toFixed(3)})`)
            .join('; ');
        this.logCommand(mapObj.frame, 'navigation_goal_dispatched', {
            waypointCount: mapObj.pointList.length,
            waypoints: serializedWaypoints,
        });
    }


    private gridToWorld(gridX: number, gridY: number, mapObj: MapObject): { x: number; y: number } {
        if (!mapObj.map) return { x: 0, y: 0 };
        const { resolution, origin } = mapObj.map;
        const offsetX = (gridX + 0.5) * resolution;
        const offsetY = (gridY + 0.5) * resolution;

        const x = origin.position.x + offsetX * mapObj.orientation.yawCos - offsetY * mapObj.orientation.yawSin;
        const y = origin.position.y + offsetX * mapObj.orientation.yawSin + offsetY * mapObj.orientation.yawCos;

        return { x, y };
    }

    normaliseMapData(rawData: unknown): Int8Array {
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
        if (rawData && typeof rawData === 'object') {
            const entries = Object.entries(rawData)
                .filter(([k]) => !Number.isNaN(Number(k)))
                .sort((a, b) => Number(a[0]) - Number(b[0]))
                .map(([, v]) => Number(v) || 0);
            if (entries.length) return Int8Array.from(entries);
        }
        console.warn('Unsupported map data format received from socket');
        return new Int8Array();
    }

    generateOccupancyGrid(rawMap: any): OccupancyGrid {
        console.log(rawMap)
        return {
        data: this.normaliseMapData(rawMap.data),
        height: rawMap?.info?.height ?? 0,
        width: rawMap?.info?.width ?? 0,
        resolution: rawMap?.info?.resolution ?? 1,
        origin: rawMap?.info?.origin ?? {
          position: { x: 0, y: 0, z: 0 },
          orientation: { x: 0, y: 0, z: 0, w: 1 },
        },
      }
    }

    updateOrientationCache(map: OccupancyGrid, orientation: Orientation): void {
        if (!map) {
            orientation.yaw = 0;
            orientation.yawCos = 1;
            orientation.yawSin = 0;
            return;
        }
        const { w, x, y, z } = map.origin.orientation;
        orientation.yaw = Math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z));
        orientation.yawCos = Math.cos(orientation.yaw);
        orientation.yawSin = Math.sin(orientation.yaw);
    }

    private drawMarkers(ctx: CanvasRenderingContext2D, mapObj: MapObject): void {
        ctx.imageSmoothingEnabled = false;
        const savedPointColor = '#1e88e5';
        const selectedPointColor = '#ff1744';
        this.drawPathThroughPoints(ctx, mapObj.pointCanvasCoords, savedPointColor);
        mapObj.pointCanvasCoords.forEach(({ x, y }) => this.drawPointMarker(ctx, x, y, savedPointColor));
        if (mapObj.selectedCanvasCoord) {
            this.drawPointMarker(ctx, mapObj.selectedCanvasCoord.x, mapObj.selectedCanvasCoord.y, selectedPointColor);
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

    private worldToCanvas(worldX: number, worldY: number, mapObj: MapObject): { x: number; y: number } | undefined {
        if (!mapObj.map) return undefined;
        const { origin, resolution, width, height } = mapObj.map;
        if (!resolution || !width || !height) return undefined;

        const dx = worldX - origin.position.x;
        const dy = worldY - origin.position.y;

        const offsetX = dx * mapObj.orientation.yawCos + dy * mapObj.orientation.yawSin;
        const offsetY = -dx * mapObj.orientation.yawSin + dy * mapObj.orientation.yawCos;

        const gridX = offsetX / resolution - 0.5;
        const gridY = offsetY / resolution - 0.5;

        if (gridX < 0 || gridX >= width || gridY < 0 || gridY >= height) return undefined;

        return { x: gridX, y: height - 1 - gridY };
    }

    private quaternionToYaw(quaternion: { x: number; y: number; z: number; w: number }): number {
        const { w, x, y, z } = quaternion;
        return Math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z));
    }

    private orientationToCanvasVector(yaw: number, mapObj: MapObject): { x: number; y: number } {
        const dirWorldX = Math.cos(yaw);
        const dirWorldY = Math.sin(yaw);
        const dirGridX = dirWorldX * mapObj.orientation.yawCos + dirWorldY * mapObj.orientation.yawSin;
        const dirGridY = -dirWorldX * mapObj.orientation.yawSin + dirWorldY * mapObj.orientation.yawCos;
        return { x: dirGridX, y: -dirGridY };
    }

    private drawRobotPoses(ctx: CanvasRenderingContext2D, mapObj: MapObject): void {
        Object.values(mapObj.robotPoses).forEach((poseData) => {
            if (!poseData) return;
            const canvasPos = this.worldToCanvas(poseData.pose.position.x, poseData.pose.position.y, mapObj);
            if (!canvasPos) return;
            const yaw = this.quaternionToYaw(poseData.pose.orientation);
            const direction = this.orientationToCanvasVector(yaw, mapObj);
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

    private logCommand(robot: string, action: string, details: MissionLogDetails): void {
        this.missionSessionService.appendLog({
            category: 'Command',
            robot,
            action,
            details,
        });
    }
}
