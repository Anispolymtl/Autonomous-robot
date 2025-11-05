import { Component, OnDestroy, OnInit, Signal } from '@angular/core';
import { CommonModule } from '@angular/common';
import { MapService } from '@app/services/map.service';
import { Map } from '@common/interfaces/map';

@Component({
  selector: 'app-map',
  standalone: true,
  imports: [CommonModule],
  templateUrl: './map.component.html',
  styleUrls: ['./map.component.scss'],
})

export class MapComponent implements OnInit, OnDestroy {

  map: Signal<Map[]>;


  constructor(
    private mapService: MapService
  ) {
    this.map = this.mapService.map.asReadonly();
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
