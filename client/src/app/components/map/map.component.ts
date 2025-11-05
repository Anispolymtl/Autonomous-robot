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

  map: {data: ArrayBuffer, height: number, width: number} | undefined;


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
}
