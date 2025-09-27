import { ComponentFixture, TestBed } from '@angular/core/testing';

import { RobotLoginComponent } from './robot-login.component';

describe('RobotLoginComponent', () => {
  let component: RobotLoginComponent;
  let fixture: ComponentFixture<RobotLoginComponent>;

  beforeEach(async () => {
    await TestBed.configureTestingModule({
      imports: [RobotLoginComponent]
    })
    .compileComponents();

    fixture = TestBed.createComponent(RobotLoginComponent);
    component = fixture.componentInstance;
    fixture.detectChanges();
  });

  it('should create', () => {
    expect(component).toBeTruthy();
  });
});
