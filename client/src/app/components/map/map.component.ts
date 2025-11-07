import { Component, OnDestroy, OnInit } from '@angular/core';
import { CommonModule } from '@angular/common';
import { MapService } from '@app/services/map.service';

@Component({
  selector: 'app-map',
  standalone: true,
  imports: [CommonModule],
  templateUrl: './map.component.html',
  styleUrls: ['./map.component.scss'],
})

export class MapComponent implements OnInit, OnDestroy {
  constructor(
    private mapService: MapService
  ) {
  }

  ngOnInit() {
    this.mapService.connectToSocket();
    if (this.mapService.isSocketAlive){
      this.mapService.resetMap();
      this.mapService.configureMapSocketFeatures();
    }
  }

  ngOnDestroy(): void {
      if (!this.mapService.isSocketAlive) {
          this.mapService.resetMap();
      }
  }

  onCanvasClick(event: MouseEvent): void {
    this.mapService.onCanvasClick(event);
  }

  addPoint(): void{
    this.mapService.addPoint();
  }

  removePoint(index: number): void {
    this.mapService.removePoint(index);
  }

  sendCoords(): void {
    this.mapService.sendCoords();
  }

  get originWorld() {
    return this.mapService.getOriginInWorld();
  }

  get selectedPoint() {
    return this.mapService.selectedPoint;
  }

  get pointList() {
    return this.mapService.pointList;
  }
}
