import { Component, OnInit } from '@angular/core';
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
  imageUrl?: string;

  constructor(private mapService: MapService) {}

  ngOnInit() {
    this.mapService.getMap().subscribe({
      next: (blob) => {
        this.imageUrl = URL.createObjectURL(blob);
      },
      error: (err) => {
        console.error('Error when loading map :', err);
      },
    });
  }
}
