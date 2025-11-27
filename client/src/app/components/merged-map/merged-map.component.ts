import { CommonModule } from '@angular/common';
import { Component, ElementRef, OnInit, ViewChild } from '@angular/core';
import { MapService } from '@app/services/map/map.service';
import { SocketService } from '@app/services/socket/socket.service';
import { OccupancyGrid } from '@common/interfaces/occupancy-grid';


@Component({
  selector: 'app-merged-map',
  standalone: true,
  imports: [CommonModule],
  templateUrl: './merged-map.component.html',
  styleUrls: ['./merged-map.component.scss'],
})
export class MergedMapComponent implements OnInit {
  @ViewChild('mergedMapCanvas', { static: true }) private mergedMapCanvas?: ElementRef<HTMLCanvasElement>;

  constructor(
    private readonly mapService: MapService,
    private readonly socketService: SocketService
  ) {}

  ngOnInit(): void {
    this.socketService.on(
      'mergedMapUpdate', (mapData: { map: unknown }) => {
        const grid: OccupancyGrid = this.mapService.generateOccupancyGrid(mapData.map);
        this.renderMergedMap(grid);
      }
    )
  }

  renderMergedMap(grid: OccupancyGrid | undefined): void {
    const canvas = this.mergedMapCanvas?.nativeElement;
    if (!grid || !canvas) return;

    const ctx = canvas.getContext('2d');
    if (!ctx) return;

    this.mapService.generateBareMap(canvas, grid, ctx);
  }
}
