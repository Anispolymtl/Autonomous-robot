import { ComponentFixture, TestBed } from '@angular/core/testing';

import { SimulationModePageComponent } from './simulation-mode-page.component';

describe('SimulationModePageComponent', () => {
  let component: SimulationModePageComponent;
  let fixture: ComponentFixture<SimulationModePageComponent>;

  beforeEach(async () => {
    await TestBed.configureTestingModule({
      imports: [SimulationModePageComponent]
    })
    .compileComponents();

    fixture = TestBed.createComponent(SimulationModePageComponent);
    component = fixture.componentInstance;
    fixture.detectChanges();
  });

  it('should create', () => {
    expect(component).toBeTruthy();
  });
});
