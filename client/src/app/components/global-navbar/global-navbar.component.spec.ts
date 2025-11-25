import { ComponentFixture, TestBed } from '@angular/core/testing';
import { NavigationEnd, Router } from '@angular/router';
import { RouterTestingModule } from '@angular/router/testing';
import { Subject } from 'rxjs';
import { GlobalNavbarComponent } from './global-navbar.component';
import { MissionModeService } from '@app/services/mission-mode/mission-mode.service';

describe('GlobalNavbarComponent', () => {
  let fixture: ComponentFixture<GlobalNavbarComponent>;
  let component: GlobalNavbarComponent;
  let router: Router;
  let routerEvents$: Subject<NavigationEnd>;
  let modeSubject$: Subject<'REAL' | 'SIMULATION' | null | undefined>;
  let originalRAF: typeof requestAnimationFrame | undefined;
  let originalCAF: typeof cancelAnimationFrame | undefined;

  beforeEach(async () => {
    routerEvents$ = new Subject<NavigationEnd>();
    modeSubject$ = new Subject<'REAL' | 'SIMULATION' | null | undefined>();

    originalRAF = window.requestAnimationFrame;
    originalCAF = window.cancelAnimationFrame;
    (window as typeof window & { requestAnimationFrame: jasmine.Spy }).requestAnimationFrame = jasmine
      .createSpy('requestAnimationFrame')
      .and.callFake((cb: FrameRequestCallback) => {
        cb(0);
        return 1;
      });
    (window as typeof window & { cancelAnimationFrame: jasmine.Spy }).cancelAnimationFrame = jasmine.createSpy(
      'cancelAnimationFrame'
    );

    await TestBed.configureTestingModule({
      imports: [GlobalNavbarComponent, RouterTestingModule],
      providers: [
        {
          provide: MissionModeService,
          useValue: {
            mode$: modeSubject$.asObservable()
          }
        }
      ]
    }).compileComponents();

    router = TestBed.inject(Router);
    spyOnProperty(router, 'events', 'get').and.returnValue(routerEvents$.asObservable());

    fixture = TestBed.createComponent(GlobalNavbarComponent);
    component = fixture.componentInstance;
    fixture.detectChanges();
  });

  afterEach(() => {
    if (originalRAF) {
      window.requestAnimationFrame = originalRAF;
    }
    if (originalCAF) {
      window.cancelAnimationFrame = originalCAF;
    }
  });

  it('crée le composant', () => {
    expect(component).toBeTruthy();
  });

  it('met à jour isHomePage selon les événements de navigation', () => {
    routerEvents$.next(new NavigationEnd(1, '/home', '/home'));
    expect(component.isHomePage).toBeTrue();

    routerEvents$.next(new NavigationEnd(2, '/real-mode', '/real-mode'));
    expect(component.isHomePage).toBeFalse();
  });

  it('réagit aux changements de mode provenant du service', () => {
    expect(component.currentMode).toBeNull();

    modeSubject$.next('SIMULATION');
    expect(component.currentMode).toBe('SIMULATION');

    modeSubject$.next('REAL');
    expect(component.currentMode).toBe('REAL');
  });

  it('calcule correctement showSimulationLink, showRealModeLink et showHomeLink', () => {
    component.isHomePage = true;
    component.currentMode = 'SIMULATION';
    expect(component.showSimulationLink).toBeFalse();

    component.isHomePage = false;
    expect(component.showSimulationLink).toBeTrue();
    expect(component.showRealModeLink).toBeFalse();

    component.currentMode = 'REAL';
    expect(component.showRealModeLink).toBeTrue();
    expect(component.showSimulationLink).toBeFalse();

    component.currentMode = null;
    expect(component.showHomeLink).toBeTrue();
    component.currentMode = 'REAL';
    expect(component.showHomeLink).toBeFalse();
  });

  it('met à jour isScrolled avec checkScroll', () => {
    const scrollSpy = spyOnProperty(window, 'scrollY', 'get');
    scrollSpy.and.returnValue(10);
    component['checkScroll']();
    expect(component.isScrolled).toBeFalse();

    scrollSpy.and.returnValue(50);
    component['checkScroll']();
    expect(component.isScrolled).toBeTrue();
  });

  it('programme une mise à jour de la hauteur de la barre de navigation', () => {
    const setPropertySpy = spyOn(document.documentElement.style, 'setProperty').and.callThrough();

    component['scheduleNavbarOffsetUpdate']();

    expect(window.requestAnimationFrame).toHaveBeenCalled();
    expect(setPropertySpy).toHaveBeenCalledWith('--navbar-offset', jasmine.stringMatching(/px$/));
  });

  it('nettoie les ressources lors de ngOnDestroy', () => {
    const destroy$ = component['destroy$'];
    const nextSpy = spyOn(destroy$, 'next').and.callThrough();
    const completeSpy = spyOn(destroy$, 'complete').and.callThrough();

    component['updateFrame'] = 123 as unknown as number;
    component.ngOnDestroy();

    expect(nextSpy).toHaveBeenCalled();
    expect(completeSpy).toHaveBeenCalled();
    expect(window.cancelAnimationFrame).toHaveBeenCalledWith(123);
  });
});
