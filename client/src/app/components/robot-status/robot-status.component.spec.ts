import { ComponentFixture, TestBed } from '@angular/core/testing';

import { RobotStatusComponent } from './robot-status.component';

describe('RobotStatusComponent', () => {
  let component: RobotStatusComponent;
  let fixture: ComponentFixture<RobotStatusComponent>;

  beforeEach(async () => {
    await TestBed.configureTestingModule({
      imports: [RobotStatusComponent]
    })
    .compileComponents();

    fixture = TestBed.createComponent(RobotStatusComponent);
    component = fixture.componentInstance;
    fixture.detectChanges();
  });

  it('should create', () => {
    expect(component).toBeTruthy();
  });
});
