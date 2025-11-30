import { CommonModule } from '@angular/common';
import { Component, ElementRef, Input, OnDestroy, OnInit, ViewChild } from '@angular/core';
import { trigger, transition, style, animate } from '@angular/animations';
import { MapCoordinate } from '@app/interfaces/map-coordinate';
import { OccupancyGrid } from '@common/interfaces/occupancy-grid';
import { Orientation } from '@app/interfaces/orientation';
import { PoseData } from '@app/interfaces/pose-data';
import { MapEvent } from '@common/enums/sockets-events';
import { MapService } from '@app/services/map/map.service';
import { SocketService } from '@app/services/socket/socket.service';

export interface Point2D {
  x: number;
  y: number;
}

type RobotId = 'limo1' | 'limo2';

export interface MapObject {
  frame: string,
  map: OccupancyGrid | undefined;
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
  animations: [
    trigger('popupAnimation', [
      transition(':enter', [
        style({ opacity: 0, transform: 'translate(-50%, calc(-100% - 15px)) scale(0.9)' }),
        animate('200ms ease-out', style({ opacity: 1, transform: 'translate(-50%, calc(-100% - 25px)) scale(1)' }))
      ]),
      transition(':leave', [
        animate('150ms ease-in', style({ opacity: 0, transform: 'translate(-50%, calc(-100% - 15px)) scale(0.9)' }))
      ])
    ])
  ]
})
export class MapComponent implements OnInit, OnDestroy {
  @ViewChild('mapCanvas', { static: true }) private mapCanvasRef?: ElementRef<HTMLCanvasElement>;
  mapObj: MapObject = this.createInitialMapObject();
  @Input({ required: true }) robotId!: RobotId;
  
  // Nouvelles propriétés pour l'UX améliorée
  hoverCoords: Point2D | undefined;
  hoverPoint: MapCoordinate | undefined;
  selectedCanvasCoords: Point2D | undefined;
  
  constructor(
    private readonly mapService: MapService,
    private readonly socketService: SocketService,
  ) {}

  ngOnInit(): void {
    if (!this.socketService.isSocketAlive()) {
      this.socketService.connect('client');
      const socket = this.socketService.getSocket;
      if (socket) {
        socket.once('connect', () => {
          this.resetMap();
          this.configureMapSocketFeatures();
        });
      }
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
    
    // Stocker les coordonnées canvas pour le popup
    const rect = canvas.getBoundingClientRect();
    this.selectedCanvasCoords = {
      x: event.clientX - rect.left,
      y: event.clientY - rect.top
    };
    
    this.mapService.onCanvasClick(event, this.mapObj, canvas);
  }

  onCanvasHover(event: MouseEvent): void {
    const canvas = this.canvas;
    if (!canvas || !this.mapObj.map) return;

    const rect = canvas.getBoundingClientRect();
    const canvasX = event.clientX - rect.left;
    const canvasY = event.clientY - rect.top;

    // Vérifier si on est bien sur le canvas
    if (canvasX < 0 || canvasY < 0 || canvasX > rect.width || canvasY > rect.height) {
      this.hoverCoords = undefined;
      this.hoverPoint = undefined;
      return;
    }

    this.hoverCoords = { x: canvasX, y: canvasY };

    // Calculer les coordonnées du point survolé
    const scaleX = canvas.width / rect.width;
    const scaleY = canvas.height / rect.height;
    const actualX = canvasX * scaleX;
    const actualY = canvasY * scaleY;

    // Convertir en coordonnées monde
    const point = this.mapService.canvasToMapCoordinate(
      this.mapObj,
      actualX,
      actualY
    );

    if (point) {
      this.hoverPoint = point;
    }
  }

  onCanvasLeave(): void {
    this.hoverCoords = undefined;
    this.hoverPoint = undefined;
  }

  clearSelection(): void {
    this.mapObj.selectedPoint = undefined;
    this.mapObj.selectedCanvasCoord = undefined;
    this.selectedCanvasCoords = undefined;
    
    // Re-render pour effacer la sélection visuelle
    if (this.canvas && this.mapObj.map) {
      this.mapService.renderMap(this.canvas, this.mapObj);
    }
  }

  addPoint(): void {
    const canvas = this.canvas;
    if (!canvas) return;
    this.mapService.sendPoint(this.mapObj);
    
    // Clear selection after adding
    this.clearSelection();
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

  // TrackBy function pour optimiser le rendu de la liste
  trackByIndex(index: number): number {
    return index;
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
      this.mapObj.map = this.mapService.generateOccupancyGrid(recoveredMap);
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
      console.log(`[${this.robotId}] Received newPoints:`, points);
      
      if (points.length === 0) {
        console.log('empty array', points);
        this.mapObj.pointList = [];
        this.mapObj.pointCanvasCoords = [];
      } else {
        // Convertir les points monde en coordonnées carte
        this.mapObj.pointList = points.map(
          (point: Point2D) => this.mapService.worldPointToMapCoordinate(this.mapObj, point.x, point.y)
        ).filter((point) => point != undefined) as MapCoordinate[];
        
        // Mettre à jour les coordonnées canvas pour le rendu
        this.mapObj.pointCanvasCoords = this.mapObj.pointList
          .map(coord => this.mapService.worldToCanvasPublic(coord.world.x, coord.world.y, this.mapObj))
          .filter(coord => coord != undefined) as Point2D[];
        
        console.log(`[${this.robotId}] Updated pointList:`, this.mapObj.pointList.length, 'points');
        console.log(`[${this.robotId}] Updated canvasCoords:`, this.mapObj.pointCanvasCoords.length, 'coords');
      }
      
      if (this.canvas && this.mapObj.map) {
        this.mapService.renderMap(this.canvas, this.mapObj);
      }
    });
  }

  private resetMap(): void {
    this.mapObj = this.createInitialMapObject();
    this.hoverCoords = undefined;
    this.hoverPoint = undefined;
    this.selectedCanvasCoords = undefined;
  }

  private createInitialMapObject(): MapObject {
    return {
      frame: this.robotId,
      map: undefined,
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