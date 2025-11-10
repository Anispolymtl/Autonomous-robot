import { CommonModule } from '@angular/common';
import { Component, ElementRef, Input, OnDestroy, OnInit, ViewChild } from '@angular/core';
import { MapCoordinate } from '@app/interfaces/map-coordinate';
import { OccupancyGrid } from '@app/interfaces/occupancy-grid';
import { Orientation } from '@app/interfaces/orientation';
import { PoseData } from '@app/interfaces/pose-data';
import { MapEvent } from '@common/enums/sockets-events';
import { MapService } from '@app/services/map.service';
import { SocketService } from '@app/services/socket.service';

export interface Point2D {
  x: number;
  y: number;
}

type RobotId = 'limo1' | 'limo2';

export interface MapObject {
  frame: string,
  map: OccupancyGrid | undefined;
  originCanvasPosition: Point2D | undefined;
  selectedCanvasCoord: Point2D | undefined;
  pointCanvasCoords: Point2D[];
  pointList: MapCoordinate[];
  robotPoses: Record<string, PoseData | undefined>;
  selectedPoint: MapCoordinate | undefined;
  orientation: Orientation;
}

@Component({
  selector: 'app-map',
  standalone: true,
  imports: [CommonModule],
  templateUrl: './map.component.html',
  styleUrls: ['./map.component.scss'],
})
export class MapComponent implements OnInit, OnDestroy {
  @ViewChild('mapCanvas', { static: true }) private mapCanvasRef?: ElementRef<HTMLCanvasElement>;
  mapObj: MapObject = this.createInitialMapObject();
  @Input({ required: true }) robotId!: RobotId;
  
  constructor(
    private readonly mapService: MapService,
    private readonly socketService: SocketService,
  ) {}

ngOnInit(): void {
    if (!this.socketService.isSocketAlive()) {
      this.socketService.connect('client');
      this.socketService.getSocket.once('connect', () => {
        this.resetMap();
        this.configureMapSocketFeatures();
      });
    } else {
      this.resetMap();
      this.configureMapSocketFeatures();
    }
  }

  ngOnDestroy(): void {
    this.resetMap();
  }

  onCanvasClick(event: MouseEvent): void {
    const canvas = this.canvas;
    if (!canvas) return;
    this.mapService.onCanvasClick(event, this.mapObj, canvas);
  }

  addPoint(): void {
    const canvas = this.canvas;
    if (!canvas) return;
    this.mapService.sendPoint(this.mapObj);
  }

  removePoint(index: number): void {
    const canvas = this.canvas;
    if (!canvas) return;
    this.mapService.removePoint(index, this.mapObj);
  }

  sendCoords(): void {
    const canvas = this.canvas;
    if (!canvas) return;
    this.mapService.sendGoal(this.mapObj);
  }

  get originWorld() {
    return this.mapObj.map ? this.mapService.getOriginInWorld(this.mapObj.map) : undefined;
  }

  get selectedPoint() {
    return this.mapObj.selectedPoint;
  }

  get pointList() {
    return this.mapObj.pointList;
  }

  private configureMapSocketFeatures(): void {
    this.socketService.on(`/${this.robotId}/${MapEvent.RecoverMap}`, (recoveredMap: any) => {
      console.log(MapEvent.RecoverMap)
      this.mapObj.map = {
        data: this.mapService.normaliseMapData(recoveredMap?.data),
        height: recoveredMap?.info?.height ?? 0,
        width: recoveredMap?.info?.width ?? 0,
        resolution: recoveredMap?.info?.resolution ?? 1,
        origin: recoveredMap?.info?.origin ?? {
          position: { x: 0, y: 0, z: 0 },
          orientation: { x: 0, y: 0, z: 0, w: 1 },
        },
      };
      this.mapService.updateOrientationCache(this.mapObj.map, this.mapObj.orientation);
      if (!this.mapObj.robotPoses[this.robotId]) {
        this.mapObj.robotPoses[this.robotId] = {
          header: { frame_id: `${this.robotId}/map`},
          pose: { position: { x: 0, y: 0, z: 0 }, orientation: { x: 0, y: 0, z: 0, w: 1 } },
        };
      }
      const canvas = this.canvas;
      if (canvas && this.mapObj.map) this.mapService.renderMap(canvas, this.mapObj);
    });

    this.socketService.on(`/${this.robotId}/${MapEvent.PoseUpdate}`, (poseData: PoseData) => {
      if (!poseData) return;
      this.mapObj.robotPoses[this.robotId] = poseData;
      if (this.canvas && this.mapObj.map) this.mapService.renderMap(this.canvas, this.mapObj);
    });

    this.socketService.on(`/${this.robotId}/${MapEvent.newPoints}`, (points: Point2D[]) => {
      if (points.length === 0) console.log('empty array', points)
      this.mapObj.pointList = points.map(
        (point: Point2D) => this.mapService.worldPointToMapCoordinate(this.mapObj, point.x, point.y)
      ).filter((point) => point != undefined);
      if (this.canvas && this.mapObj.map) this.mapService.renderMap(this.canvas, this.mapObj);
    });
  }

  private resetMap(): void {
    this.mapObj = this.createInitialMapObject();
  }

  private createInitialMapObject(): MapObject {
    return {
      frame: this.robotId,
      map: undefined,
      originCanvasPosition: undefined,
      selectedCanvasCoord: undefined,
      pointCanvasCoords: [],
      pointList: [],
      robotPoses: {},
      selectedPoint: undefined,
      orientation: { yaw: 0, yawCos: 1, yawSin: 0 },
    };
  }

  private get canvas(): HTMLCanvasElement | undefined {
    return this.mapCanvasRef?.nativeElement;
  }
}
