import { Component, OnDestroy, OnInit } from '@angular/core';
import { CommonModule } from '@angular/common';
import { MapCoordinate, MapService } from '@app/services/map.service';

@Component({
  selector: 'app-map',
  standalone: true,
  imports: [CommonModule],
  templateUrl: './map.component.html',
  styleUrls: ['./map.component.scss'],
})

export class MapComponent implements OnInit, OnDestroy {

  map = this.mapService.map;
  selectedPoint: MapCoordinate | undefined;

  constructor(
    private mapService: MapService
  ) {
    this.map = this.mapService.map;
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
    const canvas = event.target as HTMLCanvasElement | null;
    if (!canvas) return;

    const rect = canvas.getBoundingClientRect();
    if (!rect.width || !rect.height) return;

    const scaleX = canvas.width / rect.width;
    const scaleY = canvas.height / rect.height;
    const canvasX = (event.clientX - rect.left) * scaleX;
    const canvasY = (event.clientY - rect.top) * scaleY;

    const coordinate = this.mapService.canvasPointToMapCoordinate(canvasX, canvasY);
    if (coordinate) {
      this.selectedPoint = coordinate;
      console.log('Selected map coordinate:', coordinate);
    }
  }

  get originWorld() {
    return this.mapService.getOriginInWorld();
  }
}
