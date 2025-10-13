import { Component,ElementRef, OnInit, ViewChild } from '@angular/core';
import { CommonModule } from '@angular/common';
import { MapService } from '@app/services/map/map.service';

@Component({
  selector: 'app-map',
  standalone: true,
  imports: [CommonModule],
  templateUrl: './map.component.html',
  styleUrls: ['./map.component.scss'],
})

export class MapComponent implements OnInit {
  @ViewChild('canvas', { static: true }) canvasRef!: ElementRef<HTMLCanvasElement>;

  constructor(private mapService: MapService) {}

  ngOnInit() {
    this.mapService.getGMap().subscribe({
      next: ({ imageData }) => {
        const canvas = this.canvasRef.nativeElement;
        canvas.width = imageData.width;
        canvas.height = imageData.height;
        const ctx = canvas.getContext('2d');
        if (ctx) ctx.putImageData(imageData, 0, 0);
      },
      error: (err) => console.error('Error loading map:', err),
    });
  }
}
