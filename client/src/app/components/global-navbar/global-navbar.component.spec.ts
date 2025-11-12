// import { ComponentFixture, TestBed } from '@angular/core/testing';
// import { Router, NavigationEnd, ActivatedRoute } from '@angular/router';
// import { of, BehaviorSubject } from 'rxjs';
// import { GlobalNavbarComponent } from './global-navbar.component';
// import { MissionModeService, MissionMode } from '@app/services/mission-mode/mission-mode.service';

// describe('GlobalNavbarComponent', () => {
//   let component: GlobalNavbarComponent;
//   let fixture: ComponentFixture<GlobalNavbarComponent>;
//   let routerSpy: Partial<Router>;
//   let activatedRouteStub: Partial<ActivatedRoute>;
//   let missionModeServiceMock: Partial<MissionModeService>;
//   let modeSubject: BehaviorSubject<MissionMode | undefined>;

//   beforeEach(async () => {
//     modeSubject = new BehaviorSubject<MissionMode | undefined>(null);

//     missionModeServiceMock = {
//       mode$: modeSubject.asObservable(),
//     };

//     routerSpy = {
//       events: of(new NavigationEnd(0, '/home', '/home')),
//     };

//     activatedRouteStub = {
//       snapshot: {
//         url: [],
//         params: {},
//         queryParams: {},
//         fragment: null,
//         data: {},
//         outlet: 'primary',
//         routeConfig: null,
//         root: {} as any,
//         parent: null,
//         firstChild: null,
//         children: [],
//         pathFromRoot: [],
//         paramMap: {} as any,
//         queryParamMap: {} as any,
//       } as any,
//     };

//     await TestBed.configureTestingModule({
//       imports: [GlobalNavbarComponent],
//       providers: [
//         { provide: Router, useValue: routerSpy },
//         { provide: MissionModeService, useValue: missionModeServiceMock },
//         { provide: ActivatedRoute, useValue: activatedRouteStub },
//       ],
//     }).compileComponents();

//     fixture = TestBed.createComponent(GlobalNavbarComponent);
//     component = fixture.componentInstance;
//     fixture.detectChanges();
//   });

//   it('should create', () => {
//     expect(component).toBeTruthy();
//   });

//   it('should detect home page from router events', () => {
//     expect(component.isHomePage).toBeTrue();
//   });

//   it('should update currentMode when missionModeService emits', () => {
//     expect(component.currentMode).toBeNull();

//     modeSubject.next('SIMULATION');
//     fixture.detectChanges();
//     expect(component.currentMode).toBe('SIMULATION');

//     modeSubject.next('REAL');
//     fixture.detectChanges();
//     expect(component.currentMode).toBe('REAL');
//   });

//   it('should compute showSimulationLink correctly', () => {
//     component.isHomePage = true;
//     component.currentMode = 'SIMULATION';
//     expect(component.showSimulationLink).toBeFalse();

//     component.isHomePage = false;
//     component.currentMode = 'SIMULATION';
//     expect(component.showSimulationLink).toBeTrue();

//     component.currentMode = 'REAL';
//     expect(component.showSimulationLink).toBeFalse();
//   });

//   it('should compute showRealModeLink correctly', () => {
//     component.isHomePage = true;
//     component.currentMode = 'REAL';
//     expect(component.showRealModeLink).toBeFalse();

//     component.isHomePage = false;
//     component.currentMode = 'REAL';
//     expect(component.showRealModeLink).toBeTrue();

//     component.currentMode = 'SIMULATION';
//     expect(component.showRealModeLink).toBeFalse();
//   });

//   it('should compute showHomeLink correctly', () => {
//     component.currentMode = null;
//     expect(component.showHomeLink).toBeTrue();

//     component.currentMode = 'REAL';
//     expect(component.showHomeLink).toBeFalse();

//     component.currentMode = 'SIMULATION';
//     expect(component.showHomeLink).toBeFalse();
//   });

//   it('should update isScrolled on checkScroll', () => {
//     spyOnProperty(window, 'scrollY', 'get').and.returnValue(0);
//     component['checkScroll']();
//     expect(component.isScrolled).toBeFalse();

//     spyOnProperty(window, 'scrollY', 'get').and.returnValue(100);
//     component['checkScroll']();
//     expect(component.isScrolled).toBeTrue();
//   });

//   it('should complete destroy$ on ngOnDestroy', () => {
//     const destroySpy = spyOn((component as any).destroy$, 'complete');
//     component.ngOnDestroy();
//     expect(destroySpy).toHaveBeenCalled();
//   });
// });
