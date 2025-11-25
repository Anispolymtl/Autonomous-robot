import { MapService } from './map.service';
import { SocketService } from '@app/services/socket/socket.service';
import { MissionSessionService } from '@app/services/mission-session/mission-session.service';
import { MapObject } from '@app/components/map/map.component';
import { OccupancyGrid } from '@app/interfaces/occupancy-grid';
import { Orientation } from '@app/interfaces/orientation';

describe('MapService', () => {
    let service: MapService;
    let socketServiceSpy: jasmine.SpyObj<SocketService>;
    let missionSessionServiceSpy: jasmine.SpyObj<MissionSessionService>;
    let canvas: HTMLCanvasElement;
    let ctx: CanvasRenderingContext2D;

    beforeEach(() => {
        socketServiceSpy = jasmine.createSpyObj('SocketService', ['send']);
        missionSessionServiceSpy = jasmine.createSpyObj('MissionSessionService', ['appendLog']);
        service = new MapService(socketServiceSpy, missionSessionServiceSpy);

        canvas = document.createElement('canvas');
        ctx = canvas.getContext('2d') as CanvasRenderingContext2D;
        spyOn(canvas, 'getContext').and.returnValue(ctx);
    });

    it('devrait être créé', () => {
        expect(service).toBeTruthy();
    });

    it('renderMap ne fait rien si mapObj.map est manquant', () => {
        const mapObj = { map: undefined } as unknown as MapObject;
        spyOn(console, 'log');
        service.renderMap(canvas, mapObj);
        expect(console.log).not.toHaveBeenCalledWith('render');
    });

    it('renderMap met à jour la taille du canvas et trace des pixels', () => {
        const data = new Int8Array([0, 50, -1, 100]);
        const mapObj = {
            map: { data, width: 2, height: 2 },
            pointCanvasCoords: [],
            robotPoses: {},
            orientation: { yaw: 0, yawCos: 1, yawSin: 0 },
        } as unknown as MapObject;

        spyOn(ctx, 'putImageData');
        service.renderMap(canvas, mapObj);
        expect(ctx.putImageData).toHaveBeenCalled();
    });

    it('getOriginInWorld retourne la position d’origine', () => {
        const map: OccupancyGrid = {
            data: new Int8Array(),
            width: 1,
            height: 1,
            resolution: 1,
            origin: { position: { x: 5, y: 10, z: 15 }, orientation: { x: 0, y: 0, z: 0, w: 1 } },
        };
        const result = service.getOriginInWorld(map);
        expect(result).toEqual({ x: 5, y: 10 });
    });

    it('canvasPointToMapCoordinate retourne undefined si hors limites', () => {
        const mapObj = { map: { width: 10, height: 10 } } as MapObject;
        const result = service.canvasPointToMapCoordinate(mapObj, -1, 0);
        expect(result).toBeUndefined();
    });

    it('canvasPointToMapCoordinate retourne une coordonnée valide', () => {
        const mapObj = {
            map: {
                width: 10,
                height: 10,
                resolution: 1,
                origin: { position: { x: 0, y: 0 }, orientation: { x: 0, y: 0, z: 0, w: 1 } },
            },
            orientation: { yaw: 0, yawCos: 1, yawSin: 0 },
        } as MapObject;
        const result = service.canvasPointToMapCoordinate(mapObj, 5, 5);
        expect(result?.cell).toEqual({ x: 5, y: 4 });
    });

    it('worldPointToMapCoordinate retourne undefined si hors limites', () => {
        const mapObj = {
            map: { width: 1, height: 1, resolution: 1, origin: { position: { x: 0, y: 0 }, orientation: { x:0,y:0,z:0,w:1 } } },
            orientation: { yawCos: 1, yawSin: 0 },
        } as MapObject;
        const result = service.worldPointToMapCoordinate(mapObj, 999, 999);
        expect(result).toBeUndefined();
    });

    it('sendPoint envoie les données du point et réinitialise la sélection', () => {
        const mapObj = {
            frame: 'limo1',
            selectedPoint: { world: { x: 1, y: 2 }, cell: { x: 3, y: 4 } },
            selectedCanvasCoord: { x: 10, y: 20 },
        } as unknown as MapObject;

        service.sendPoint(mapObj);
        expect(socketServiceSpy.send).toHaveBeenCalledWith('point', jasmine.anything());
        expect(missionSessionServiceSpy.appendLog).toHaveBeenCalled();
        expect(mapObj.selectedPoint).toBeUndefined();
    });

    it('removePoint envoie removePoint et logCommand', () => {
        const mapObj = {
            frame: 'limo1',
            pointList: [{ world: { x: 1, y: 2 } }],
        } as unknown as MapObject;

        service.removePoint(0, mapObj);
        expect(socketServiceSpy.send).toHaveBeenCalledWith('removePoint', jasmine.anything());
        expect(missionSessionServiceSpy.appendLog).toHaveBeenCalled();
    });

    it('sendGoal ne fait rien si pointList vide', () => {
        const mapObj = { frame: 'limo1', pointList: [] } as unknown as MapObject;
        spyOn(console, 'log');
        service.sendGoal(mapObj);
        expect(console.log).toHaveBeenCalledWith('No coordinates to send.');
    });

    it('normaliseMapData gère plusieurs formats', () => {
        expect(service.normaliseMapData(new Int8Array([1]))).toEqual(new Int8Array([1]));
        expect(service.normaliseMapData(new ArrayBuffer(2))).toBeInstanceOf(Int8Array);
        expect(service.normaliseMapData({ data: [1, 2] })).toEqual(new Int8Array([1, 2]));
        expect(service.normaliseMapData([1, 2])).toEqual(new Int8Array([1, 2]));
    });

    it('updateOrientationCache met à jour les angles yawCos/yawSin', () => {
        const orientation: Orientation = { yaw: 0, yawCos: 0, yawSin: 0 };
        const map: OccupancyGrid = {
            data: new Int8Array(),
            width: 1,
            height: 1,
            resolution: 1,
            origin: { position: { x: 0, y: 0, z: 15 }, orientation: { x: 0, y: 0, z: 0.707, w: 0.707 } },
        };
        service.updateOrientationCache(map, orientation);
        expect(orientation.yawCos).toBeCloseTo(Math.cos(Math.PI / 2), 1);
    });

    it('updateOrientationCache remet les valeurs par défaut si map undefined', () => {
        const orientation: Orientation = { yaw: 1, yawCos: 0.5, yawSin: 0.5 };
        service.updateOrientationCache(undefined as any, orientation);
        expect(orientation).toEqual({ yaw: 0, yawCos: 1, yawSin: 0 });
    });
});
