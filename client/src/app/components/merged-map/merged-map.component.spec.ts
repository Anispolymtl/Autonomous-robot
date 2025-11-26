import { ElementRef } from '@angular/core';
import { ComponentFixture, TestBed } from '@angular/core/testing';
import { MapService } from '@app/services/map/map.service';
import { OccupancyGrid } from '@common/interfaces/occupancy-grid';
import { MergedMapComponent } from './merged-map.component';

describe('MergedMapComponent', () => {
  let component: MergedMapComponent;
  let fixture: ComponentFixture<MergedMapComponent>;
  let mapService: jasmine.SpyObj<MapService>;

  beforeEach(async () => {
    mapService = jasmine.createSpyObj('MapService', ['generateBareMap']);

    await TestBed.configureTestingModule({
      imports: [MergedMapComponent],
      providers: [{ provide: MapService, useValue: mapService }],
    }).compileComponents();

    fixture = TestBed.createComponent(MergedMapComponent);
    component = fixture.componentInstance;
    fixture.detectChanges();
  });

  it('should create', () => {
    expect(component).toBeTruthy();
  });

  it('should render merged map when canvas and grid are provided', () => {
    const testCanvas = document.createElement('canvas');
    const ctx = testCanvas.getContext('2d') as CanvasRenderingContext2D;
    spyOn(testCanvas, 'getContext').and.returnValue(ctx);

    (component as unknown as { mergedMapCanvas?: ElementRef<HTMLCanvasElement> }).mergedMapCanvas = {
      nativeElement: testCanvas,
    } as ElementRef<HTMLCanvasElement>;

    const grid: OccupancyGrid = {
      data: new Int8Array([0]),
      width: 1,
      height: 1,
      resolution: 1,
      origin: { position: { x: 0, y: 0, z: 0 }, orientation: { x: 0, y: 0, z: 0, w: 1 } },
    };

    component.renderMergedMap(grid);

    expect(mapService.generateBareMap).toHaveBeenCalledWith(testCanvas, grid, ctx);
  });
});
