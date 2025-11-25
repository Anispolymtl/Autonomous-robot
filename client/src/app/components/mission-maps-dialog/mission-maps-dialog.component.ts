import { CommonModule } from '@angular/common';
import { AfterViewInit, Component, ElementRef, Inject, ViewChild } from '@angular/core';
import { MAT_DIALOG_DATA, MatDialogRef } from '@angular/material/dialog';
import { Mission } from '@app/interfaces/mission';
import { MapService } from '@app/services/map/map.service';
import { OccupancyGrid } from '@common/interfaces/occupancy-grid';

@Component({
    selector: 'app-mission-maps-dialog',
    standalone: true,
    imports: [CommonModule],
    templateUrl: './mission-maps-dialog.component.html',
    styleUrls: ['./mission-maps-dialog.component.scss']
})
export class MissionMapsDialogComponent implements AfterViewInit {
    @ViewChild('mapCanvasPrimary', { static: true }) primaryCanvas?: ElementRef<HTMLCanvasElement>;
    @ViewChild('mapCanvasSecondary', { static: true }) secondaryCanvas?: ElementRef<HTMLCanvasElement>;

    occupancyGrid1: OccupancyGrid | undefined;
    occupancyGrid2: OccupancyGrid | undefined;

    constructor(
        private dialogRef: MatDialogRef<MissionMapsDialogComponent>,
        private readonly mapService: MapService,
        @Inject(MAT_DIALOG_DATA) public data: { mission: Mission }
    ) {}

    ngAfterViewInit(): void {
        this.renderMissionMaps();
    }

    close(): void {
        this.dialogRef.close();
    }

    private renderMissionMaps(): void {
        const primaryRaw = this.data.mission.maps?.limo1;
        const secondaryRaw = this.data.mission.maps?.limo2;

        this.occupancyGrid1 = this.renderSingleMap(primaryRaw, this.primaryCanvas?.nativeElement);
        this.occupancyGrid2 = this.renderSingleMap(secondaryRaw, this.secondaryCanvas?.nativeElement);
    }

    private renderSingleMap(rawMap: unknown, canvas?: HTMLCanvasElement): OccupancyGrid | undefined {
        if (!rawMap || !canvas) return undefined;
        console.log(rawMap)
        const ctx = canvas.getContext('2d');
        if (!ctx) return undefined;
        const grid = this.mapService.generateOccupancyGrid(rawMap);
        if (!grid.data?.length || !grid.width || !grid.height) return undefined;
        this.mapService.generateBareMap(canvas, grid, ctx);
        console.log(grid);
        console.log(grid.data);
        return grid;
    }
}
