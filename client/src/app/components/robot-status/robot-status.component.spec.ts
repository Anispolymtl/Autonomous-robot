import { ComponentFixture, TestBed } from '@angular/core/testing';
import { RobotStatusComponent } from './robot-status.component';
import { MissionStateService } from '@app/services/state/state.service';
import { Subject } from 'rxjs';

describe('RobotStatusComponent', () => {
  let component: RobotStatusComponent;
  let fixture: ComponentFixture<RobotStatusComponent>;
  let missionStateServiceSpy: jasmine.SpyObj<MissionStateService>;

  let limo1State$: Subject<string>;
  let limo2State$: Subject<string>;

  beforeEach(async () => {
    limo1State$ = new Subject<string>();
    limo2State$ = new Subject<string>();

    missionStateServiceSpy = jasmine.createSpyObj('MissionStateService', [
      'connectToSocket',
      'disconnect',
      'getLimo1State$',
      'getLimo2State$'
    ]);

    missionStateServiceSpy.getLimo1State$.and.returnValue(limo1State$.asObservable());
    missionStateServiceSpy.getLimo2State$.and.returnValue(limo2State$.asObservable());

    await TestBed.configureTestingModule({
      imports: [RobotStatusComponent],
      providers: [
        { provide: MissionStateService, useValue: missionStateServiceSpy }
      ]
    }).compileComponents();

    fixture = TestBed.createComponent(RobotStatusComponent);
    component = fixture.componentInstance;
  });

  it('should create', () => {
    expect(component).toBeTruthy();
    expect(component.robot1Status).toBe('En attente');
    expect(component.robot2Status).toBe('En attente');
    expect(component.robot1Battery).toBe(21);
    expect(component.robot2Battery).toBe(67);
  });

  it('should connect to socket and subscribe to robot states on init', () => {
    component.ngOnInit();

    expect(missionStateServiceSpy.connectToSocket).toHaveBeenCalled();

    limo1State$.next('En mission');
    limo2State$.next('Erreur');

    expect(component.robot1Status).toBe('En mission');
    expect(component.robot2Status).toBe('Erreur');
  });

  it('should unsubscribe and disconnect on destroy', () => {
    component.ngOnInit();
    spyOn(component['sub1'], 'unsubscribe');
    spyOn(component['sub2'], 'unsubscribe');

    component.ngOnDestroy();

    expect(component['sub1'].unsubscribe).toHaveBeenCalled();
    expect(component['sub2'].unsubscribe).toHaveBeenCalled();
    expect(missionStateServiceSpy.disconnect).toHaveBeenCalled();
  });
});
