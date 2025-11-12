import { ComponentFixture, TestBed } from '@angular/core/testing';
import { MapComponent } from './map.component';
import { MapService } from '@app/services/map/map.service';
import { SocketService } from '@app/services/socket/socket.service';

class MockMapService {
  onCanvasClick = jasmine.createSpy('onCanvasClick');
  sendPoint = jasmine.createSpy('sendPoint');
  removePoint = jasmine.createSpy('removePoint');
  sendGoal = jasmine.createSpy('sendGoal');
  getOriginInWorld = jasmine.createSpy('getOriginInWorld').and.returnValue(undefined);
  normaliseMapData = jasmine.createSpy('normaliseMapData').and.callFake((d) => d);
  updateOrientationCache = jasmine.createSpy('updateOrientationCache');
  renderMap = jasmine.createSpy('renderMap');
  worldPointToMapCoordinate = jasmine.createSpy('worldPointToMapCoordinate').and.returnValue({ x: 0, y: 0 });
}

class MockSocket {
  once = jasmine.createSpy('once');
  on = jasmine.createSpy('on');
}

class MockSocketService {
  getSocket = new MockSocket();
  isSocketAlive = jasmine.createSpy('isSocketAlive').and.returnValue(true);
  connect = jasmine.createSpy('connect');
  on = jasmine.createSpy('on');
}

describe('MapComponent', () => {
  let component: MapComponent;
  let fixture: ComponentFixture<MapComponent>;

  beforeEach(async () => {
    await TestBed.configureTestingModule({
      imports: [MapComponent],
      providers: [
        { provide: MapService, useClass: MockMapService },
        { provide: SocketService, useClass: MockSocketService },
      ],
    }).compileComponents();

    fixture = TestBed.createComponent(MapComponent);
    component = fixture.componentInstance;
    component.robotId = 'limo1';
    fixture.detectChanges();
  });

  it('should create the component', () => {
    expect(component).toBeTruthy();
  });
});
