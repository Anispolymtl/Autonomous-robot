// import { TestBed } from '@angular/core/testing';
// import { MapService } from './map.service';
// import { SocketService } from './socket.service';
// import { MapObject } from '@app/interfaces/map-object';
// import { OccupancyGrid } from '@app/interfaces/occupancy-grid';
// import { Orientation } from '@app/interfaces/orientation';

// describe('MapService', () => {
//   let service: MapService;
//   let mockSocketService: any;

//   beforeEach(() => {
//     mockSocketService = {
//       send: jasmine.createSpy('send'),
//     };

//     TestBed.configureTestingModule({
//       providers: [
//         MapService,
//         { provide: SocketService, useValue: mockSocketService },
//       ],
//     });

//     service = TestBed.inject(MapService);
//   });

//   it('should be created', () => {
//     expect(service).toBeTruthy();
//   });

//   it('should render map and set originCanvasPosition', () => {
//     const canvas = document.createElement('canvas');
//     const mapObj: MapObject = {
//       frame: 'limo1',
//       map: {
//         width: 2,
//         height: 2,
//         resolution: 1,
//         origin: { position: { x: 0, y: 0, z: 0 }, orientation: { x: 0, y: 0, z: 0, w: 1 } },
//         data: [0, 50, 100, -1],
//       } as OccupancyGrid,
//       originCanvasPosition: undefined,
//       selectedCanvasCoord: undefined,
//       pointCanvasCoords: [],
//       pointList: [],
//       robotPoses: {},
//       selectedPoint: undefined,
//       orientation: { yaw: 0, yawCos: 1, yawSin: 0 } as Orientation,
//     };

//     service.renderMap(canvas, mapObj);

//     expect(mapObj.originCanvasPosition).toEqual({ x: 0.5, y: 1.5 });
//   });

//   it('should convert canvas point to map coordinate', () => {
//     const mapObj: MapObject = {
//       frame: 'limo1',
//       map: {
//         width: 2,
//         height: 2,
//         resolution: 1,
//         origin: { position: { x: 0, y: 0, z: 0 }, orientation: { x: 0, y: 0, z: 0, w: 1 } },
//         data: [0, 50, 100, -1],
//       } as OccupancyGrid,
//       originCanvasPosition: undefined,
//       selectedCanvasCoord: undefined,
//       pointCanvasCoords: [],
//       pointList: [],
//       robotPoses: {},
//       selectedPoint: undefined,
//       orientation: { yaw: 0, yawCos: 1, yawSin: 0 } as Orientation,
//     };

//     const coord = service.canvasPointToMapCoordinate(mapObj, 0, 0);
//     expect(coord).toBeDefined();
//     expect(coord?.cell).toEqual({ x: 0, y: 1 });
//   });

//   it('should send a point via socket', () => {
//     const mapObj: MapObject = {
//       frame: 'limo1',
//       map: undefined,
//       originCanvasPosition: undefined,
//       selectedCanvasCoord: { x: 1, y: 1 },
//       pointCanvasCoords: [],
//       pointList: [],
//       robotPoses: {},
//       selectedPoint: { cell: { x: 0, y: 0 }, world: { x: 0, y: 0 } },
//       orientation: { yaw: 0, yawCos: 1, yawSin: 0 } as Orientation,
//     };

//     service.sendPoint(mapObj);

//     expect(mockSocketService.send).toHaveBeenCalledWith('point', { robot: 'limo1', point: { x: 0, y: 0 } });
//     expect(mapObj.selectedPoint).toBeUndefined();
//     expect(mapObj.selectedCanvasCoord).toBeUndefined();
//   });

//   it('should remove a point via socket', () => {
//     const mapObj: MapObject = {
//       frame: 'limo1',
//       map: undefined,
//       originCanvasPosition: undefined,
//       selectedCanvasCoord: undefined,
//       pointCanvasCoords: [],
//       pointList: [],
//       robotPoses: {},
//       selectedPoint: undefined,
//       orientation: { yaw: 0, yawCos: 1, yawSin: 0 } as Orientation,
//     };

//     mapObj.pointList = [{ cell: { x: 0, y: 0 }, world: { x: 0, y: 0 } }];
//     service.removePoint(0, mapObj);

//     expect(mockSocketService.send).toHaveBeenCalledWith('removePoint', { robot: 'limo1', index: 0 });
//   });
// });
