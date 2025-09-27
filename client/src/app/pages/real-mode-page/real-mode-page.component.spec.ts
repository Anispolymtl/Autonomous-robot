import { ComponentFixture, TestBed } from '@angular/core/testing';

import { RealModePageComponent } from './real-mode-page.component';

describe('RealModePageComponent', () => {
  let component: RealModePageComponent;
  let fixture: ComponentFixture<RealModePageComponent>;

  beforeEach(async () => {
    await TestBed.configureTestingModule({
      imports: [RealModePageComponent]
    })
    .compileComponents();

    fixture = TestBed.createComponent(RealModePageComponent);
    component = fixture.componentInstance;
    fixture.detectChanges();
  });

  it('should create', () => {
    expect(component).toBeTruthy();
  });
});
