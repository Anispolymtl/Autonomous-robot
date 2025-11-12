import { ComponentFixture, TestBed } from '@angular/core/testing';
import { MapComponent } from './map.component';
import { MapService } from '@app/services/map/map.service';
import { SocketService } from '@app/services/socket/socket.service';
import { MapEvent } from '@common/enums/sockets-events';
import { PoseData } from '@app/interfaces/pose-data';

type EventHandler = (payload?: unknown) => void;

describe('MapComponent', () => {
  let component: MapComponent;
  let fixture: ComponentFixture<MapComponent>;
  let mapService: jasmine.SpyObj<MapService>;
  let socketService: jasmine.SpyObj<SocketService>;
  let socketHandlers: Record<string, EventHandler[]>;
  let socketOnceHandlers: Record<string, (() => void)[]>;
  let mockSocket: { once: jasmine.Spy };

  const triggerOnce = (event: string) => {
    (socketOnceHandlers[event] || []).forEach((handler) => handler());
  };

  beforeEach(async () => {
    mapService = jasmine.createSpyObj('MapService', [
      'onCanvasClick',
      'sendPoint',
      'removePoint',
      'sendGoal',
      'getOriginInWorld',
      'normaliseMapData',
      'updateOrientationCache',
      'renderMap',
      'worldPointToMapCoordinate'
    ]);
    mapService.getOriginInWorld.and.returnValue(undefined);
    mapService.normaliseMapData.and.callFake((data) =>
      data instanceof Int8Array ? data : new Int8Array((data as number[]) || [])
    );
    mapService.worldPointToMapCoordinate.and.returnValue({
      cell: { x: 0, y: 0 },
      world: { x: 0, y: 0 }
    } as any);

    socketService = jasmine.createSpyObj('SocketService', ['isSocketAlive', 'connect', 'on', 'off', 'once']);
    socketHandlers = {};
    socketOnceHandlers = {};
    socketService.isSocketAlive.and.returnValue(true);
    socketService.on.and.callFake(<T>(event: string, handler: (data: T) => void) => {
      (socketHandlers[event] ||= []).push(handler as EventHandler);
    });
    socketService.once.and.callFake((event: string, handler: () => void) => {
      (socketOnceHandlers[event] ||= []).push(handler);
    });

    mockSocket = {
      once: jasmine.createSpy('socket.once').and.callFake((event: string, handler: () => void) => {
        (socketOnceHandlers[event] ||= []).push(handler);
      })
    };
    Object.defineProperty(socketService, 'getSocket', {
      get: () => mockSocket
    });

    await TestBed.configureTestingModule({
      imports: [MapComponent],
      providers: [
        { provide: MapService, useValue: mapService },
        { provide: SocketService, useValue: socketService }
      ]
    }).compileComponents();

    fixture = TestBed.createComponent(MapComponent);
    component = fixture.componentInstance;
    component.robotId = 'limo1';
  });

  const canvas = () => component['mapCanvasRef']?.nativeElement as HTMLCanvasElement;

  it('crée le composant', () => {
    socketService.isSocketAlive.and.returnValue(true);
    fixture.detectChanges();
    expect(component).toBeTruthy();
  });

  it('configure la carte immédiatement si le socket est actif', () => {
    socketService.isSocketAlive.and.returnValue(true);
    const resetSpy = spyOn<any>(component, 'resetMap').and.callThrough();
    const configSpy = spyOn<any>(component, 'configureMapSocketFeatures').and.callThrough();

    fixture.detectChanges();

    expect(resetSpy).toHaveBeenCalled();
    expect(configSpy).toHaveBeenCalled();
  });

  it('attend la connexion du socket avant de configurer la carte', () => {
    socketService.isSocketAlive.and.returnValue(false);
    const configSpy = spyOn<any>(component, 'configureMapSocketFeatures').and.callThrough();

    fixture.detectChanges();

    expect(socketService.connect).toHaveBeenCalledWith('client');
    expect(mockSocket.once).toHaveBeenCalledWith('connect', jasmine.any(Function));

    triggerOnce('connect');
    expect(configSpy).toHaveBeenCalled();
  });

  it('délègue les interactions du canvas au MapService', () => {
    socketService.isSocketAlive.and.returnValue(true);
    fixture.detectChanges();
    const testCanvas = canvas();
    const event = new MouseEvent('click');

    component.onCanvasClick(event);
    expect(mapService.onCanvasClick).toHaveBeenCalledWith(event, component.mapObj, testCanvas);

    component.addPoint();
    expect(mapService.sendPoint).toHaveBeenCalledWith(component.mapObj);

    component.removePoint(0);
    expect(mapService.removePoint).toHaveBeenCalledWith(0, component.mapObj);

    component.sendCoords();
    expect(mapService.sendGoal).toHaveBeenCalledWith(component.mapObj);
  });

  it('retourne l’origine dans le repère monde lorsque disponible', () => {
    socketService.isSocketAlive.and.returnValue(true);
    fixture.detectChanges();
    const origin = { x: 1, y: 2 };
    mapService.getOriginInWorld.and.returnValue(origin);
    component['mapObj'].map = {} as any;

    expect(component.originWorld).toBe(origin);
  });

  it('met à jour l’occupation lors d’un événement RecoverMap', () => {
    socketService.isSocketAlive.and.returnValue(true);
    fixture.detectChanges();
    mapService.renderMap.calls.reset();

    const recoverEvent = `/${component.robotId}/${MapEvent.RecoverMap}`;
    const handler = socketHandlers[recoverEvent][0];
    const payload = {
      data: [0, 100],
      info: {
        height: 2,
        width: 3,
        resolution: 0.5,
        origin: { position: { x: 1, y: 1, z: 0 }, orientation: { x: 0, y: 0, z: 0, w: 1 } }
      }
    };

    handler(payload);

    expect(mapService.normaliseMapData).toHaveBeenCalledWith(payload.data);
    expect(mapService.updateOrientationCache).toHaveBeenCalled();
    expect(component.mapObj.map?.width).toBe(3);
    expect(component.mapObj.robotPoses['limo1']).toBeDefined();
    expect(mapService.renderMap).toHaveBeenCalledWith(canvas(), component.mapObj);
  });

  it('actualise la pose du robot lors de PoseUpdate', () => {
    socketService.isSocketAlive.and.returnValue(true);
    fixture.detectChanges();
    component.mapObj.map = {
      data: new Int8Array(),
      height: 1,
      width: 1,
      resolution: 1,
      origin: {
        position: { x: 0, y: 0, z: 0 },
        orientation: { x: 0, y: 0, z: 0, w: 1 }
      }
    };
    mapService.renderMap.calls.reset();

    const poseHandler = socketHandlers[`/${component.robotId}/${MapEvent.PoseUpdate}`][0];
    const pose: PoseData = {
      header: { frame_id: 'frame' },
      pose: { position: { x: 1, y: 2, z: 0 }, orientation: { x: 0, y: 0, z: 0, w: 1 } }
    };

    poseHandler(pose);

    expect(component.mapObj.robotPoses['limo1']).toBe(pose);
    expect(mapService.renderMap).toHaveBeenCalledWith(canvas(), component.mapObj);
  });

  it('convertit les nouveaux points et rafraîchit l’affichage', () => {
    socketService.isSocketAlive.and.returnValue(true);
    fixture.detectChanges();
    component.mapObj.map = {
      data: new Int8Array(),
      height: 1,
      width: 1,
      resolution: 1,
      origin: {
        position: { x: 0, y: 0, z: 0 },
        orientation: { x: 0, y: 0, z: 0, w: 1 }
      }
    };
    mapService.worldPointToMapCoordinate.and.returnValues(
      {
        cell: { x: 1, y: 1 },
        world: { x: 0.1, y: 0.2 }
      } as any,
      undefined as any
    );
    mapService.renderMap.calls.reset();

    const pointsHandler = socketHandlers[`/${component.robotId}/${MapEvent.newPoints}`][0];
    pointsHandler([
      { x: 10, y: 10 },
      { x: 20, y: 20 }
    ]);

    expect(component.pointList.length).toBe(1);
    expect(mapService.worldPointToMapCoordinate).toHaveBeenCalledTimes(2);
    expect(mapService.renderMap).toHaveBeenCalledWith(canvas(), component.mapObj);
  });
});
