import { ComponentFixture, TestBed } from '@angular/core/testing';
import { MainPageComponent } from './main-page.component';
import { Component } from '@angular/core';

@Component({
  selector: 'app-robot-login',
  template: '<div>Robot Login Stub</div>'
})
class RobotLoginStubComponent {}

describe('MainPageComponent', () => {
  let component: MainPageComponent;
  let fixture: ComponentFixture<MainPageComponent>;

  beforeEach(async () => {
    await TestBed.configureTestingModule({
      imports: [
        MainPageComponent,
      ],
    })
    .overrideComponent(MainPageComponent, {
      set: { imports: [RobotLoginStubComponent] }
    })
    .compileComponents();
  });

  beforeEach(() => {
    fixture = TestBed.createComponent(MainPageComponent);
    component = fixture.componentInstance;
    fixture.detectChanges();
  });

  it('should create the MainPageComponent', () => {
    expect(component).toBeTruthy();
  });

  it('should render RobotLoginComponent', () => {
    const compiled = fixture.nativeElement as HTMLElement;
    expect(compiled.querySelector('app-robot-login')).toBeTruthy();
  });
});
