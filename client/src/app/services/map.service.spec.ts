import { TestBed } from '@angular/core/testing';
import { MapService } from './map.service';
import { SocketService } from '@app/services/socket.service';
import { MapObject } from '@app/components/map/map.component';
import { OccupancyGrid } from '@app/interfaces/occupancy-grid';
import { Orientation } from '@app/interfaces/orientation';

describe('MapService', () => {
  let service: MapService;
  let socketService: jasmine.SpyObj<SocketService>;
  let canvas: HTMLCanvasElement;
  let ctx: CanvasRenderingContext2D;

  beforeEach(() => {
    socketService = jasmine.createSpyObj('SocketService', ['send']);
    TestBed.configureTestingModule({
      providers: [
        MapService,
        { provide: SocketService, useValue: socketService },
      ],
    });

    service = TestBed.inject(MapService);
    canvas = document.createElement('canvas');
    canvas.width = 2;
    canvas.height = 2;
    ctx = canvas.getContext('2d') as CanvasRenderingContext2D;
  });

  const mockMapObj = (): MapObject => ({
    map: {
      data: new Int8Array([0, 100, 200, -1]),
      width: 2,
      height: 2,
      resolution: 1,
      origin: {
        position: { x: 0, y: 0, z: 0 },
        orientation: { x: 0, y: 0, z: 0, w: 1 },
      },
    },
    orientation: { yaw: 0, yawCos: 1, yawSin: 0 },
    pointList: [],
    pointCanvasCoords: [],
    robotPoses: {},
    frame: 'robot1',
  } as unknown as MapObject);

  it('should be created', () => {
    expect(service).toBeTruthy();
  });

  it('should normalize various data types correctly', () => {
    const array = [1, 2, 3];
    const int8 = new Int8Array(array);
    const buffer = int8.buffer;

    expect(service.normaliseMapData({ data: int8 })).toEqual(int8);
    expect(service.normaliseMapData(buffer)).toEqual(jasmine.any(Int8Array));
    expect(service.normaliseMapData(int8)).toEqual(int8);
    expect(service.normaliseMapData(array)).toEqual(Int8Array.from(array));
  });

  it('should return undefined if map is missing in getOriginInWorld', () => {
    expect(service.getOriginInWorld(undefined as unknown as OccupancyGrid)).toBeUndefined();
  });

  it('should return correct origin from getOriginInWorld', () => {
    const map: OccupancyGrid = {
      data: new Int8Array(),
      width: 1,
      height: 1,
      resolution: 1,
      origin: {
        position: { x: 10, y: 20, z: 0 },
        orientation: { x: 0, y: 0, z: 0, w: 1 },
      },
    };
    expect(service.getOriginInWorld(map)).toEqual({ x: 10, y: 20 });
  });

  it('should update orientation cache correctly', () => {
    const map: OccupancyGrid = {
      data: new Int8Array(),
      width: 1,
      height: 1,
      resolution: 1,
      origin: { position: { x: 0, y: 0, z: 0 }, orientation: { w: 1, x: 0, y: 0, z: 0 } },
    };
    const orientation: Orientation = { yaw: 0, yawCos: 0, yawSin: 0 };
    service.updateOrientationCache(map, orientation);
    expect(orientation.yawCos).toBeCloseTo(1);
    expect(orientation.yawSin).toBeCloseTo(0);
  });

  it('should handle missing map gracefully in worldPointToMapCoordinate', () => {
    const mapObj = mockMapObj();
    mapObj.map = undefined as any;
    const result = service.worldPointToMapCoordinate(mapObj, 1, 1);
    expect(result).toBeUndefined();
  });

  it('should convert between canvas and map coordinates', () => {
    const mapObj = mockMapObj();
    const coord = service.canvasPointToMapCoordinate(mapObj, 1, 1);
    expect(coord?.cell).toEqual({ x: 1, y: 0 });
    expect(coord?.world).toBeDefined();
  });

  it('should ignore out-of-bounds canvas points', () => {
    const mapObj = mockMapObj();
    expect(service.canvasPointToMapCoordinate(mapObj, -1, -1)).toBeUndefined();
  });

  it('should send point when selected', () => {
    const mapObj = mockMapObj();
    mapObj.selectedPoint = { cell: { x: 1, y: 1 }, world: { x: 1, y: 1 } };
    mapObj.selectedCanvasCoord = { x: 1, y: 1 };
    service.sendPoint(mapObj);
    expect(socketService.send).toHaveBeenCalledWith('point', {
      robot: 'robot1',
      point: { x: 1, y: 1 },
    });
  });

  it('should not send point when nothing is selected', () => {
    const mapObj = mockMapObj();
    service.sendPoint(mapObj);
    expect(socketService.send).not.toHaveBeenCalled();
  });

  it('should send removePoint with valid index', () => {
    const mapObj = mockMapObj();
    mapObj.pointList = [{}, {}] as any;
    service.removePoint(1, mapObj);
    expect(socketService.send).toHaveBeenCalledWith('removePoint', {
      robot: 'robot1',
      index: 1,
    });
  });

  it('should not send removePoint for invalid index', () => {
    const mapObj = mockMapObj();
    mapObj.pointList = [];
    service.removePoint(5, mapObj);
    expect(socketService.send).not.toHaveBeenCalled();
  });

  it('should sendGoal if pointList not empty', () => {
    const mapObj = mockMapObj();
    mapObj.pointList = [{ x: 1, y: 2 }] as any;
    service.sendGoal(mapObj);
    expect(socketService.send).toHaveBeenCalledWith('startNavGoal', { robot: 'robot1' });
  });

  it('should skip sendGoal if pointList empty', () => {
    const mapObj = mockMapObj();
    service.sendGoal(mapObj);
    expect(socketService.send).not.toHaveBeenCalled();
  });

  it('should not crash when rendering map with valid map data', () => {
    const mapObj = mockMapObj();
    spyOn(canvas, 'getContext').and.returnValue(ctx);
    expect(() => service.renderMap(canvas, mapObj)).not.toThrow();
  });
});
