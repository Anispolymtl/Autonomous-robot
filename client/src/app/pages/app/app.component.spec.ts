// import { ComponentFixture, TestBed, fakeAsync, tick } from '@angular/core/testing';
// import { AppComponent } from './app.component';
// import { Router, NavigationEnd } from '@angular/router';
// import { Subject, of } from 'rxjs';
// import { MissionModeService } from '@app/services/mission-mode.service';
// import { MissionSessionService } from '@app/services/mission-session.service';
// import { GlobalNavbarComponent } from '@app/components/global-navbar/global-navbar.component';

// describe('AppComponent', () => {
//   let component: AppComponent;
//   let fixture: ComponentFixture<AppComponent>;

//   let routerSpy: jasmine.SpyObj<Router>;
//   let missionModeSpy: jasmine.SpyObj<MissionModeService>;
//   let missionSessionSpy: jasmine.SpyObj<MissionSessionService>;
//   let modeSubject: Subject<'REAL' | 'SIMULATION' | null>;

//   beforeEach(async () => {
//     modeSubject = new Subject<'REAL' | 'SIMULATION' | null>();

//     routerSpy = jasmine.createSpyObj('Router', ['navigate'], {
//       events: of(new NavigationEnd(0, '/home', '/home')),
//     });
//     // Définit url read-only
//     Object.defineProperty(routerSpy, 'url', { get: () => '/home' });

//     missionModeSpy = jasmine.createSpyObj('MissionModeService', ['startPolling', 'ensureModeLoaded'], {
//       mode$: modeSubject.asObservable(),
//     });

//     missionSessionSpy = jasmine.createSpyObj('MissionSessionService', ['rehydrateActiveMission']);

//     await TestBed.configureTestingModule({
//       imports: [AppComponent, GlobalNavbarComponent],
//       providers: [
//         { provide: Router, useValue: routerSpy },
//         { provide: MissionModeService, useValue: missionModeSpy },
//         { provide: MissionSessionService, useValue: missionSessionSpy },
//       ],
//     }).compileComponents();

//     fixture = TestBed.createComponent(AppComponent);
//     component = fixture.componentInstance;
//   });

//   it('should create', () => {
//     expect(component).toBeTruthy();
//   });

//   it('should start polling on init', () => {
//     fixture.detectChanges();
//     expect(missionModeSpy.startPolling).toHaveBeenCalled();
//   });

//   it('should rehydrate mission and redirect based on ensureModeLoaded', fakeAsync(() => {
//     missionModeSpy.ensureModeLoaded.and.returnValue(Promise.resolve('SIMULATION'));
//     fixture.detectChanges();
//     tick(); // attend la promise
//     expect(missionSessionSpy.rehydrateActiveMission).toHaveBeenCalled();
//     expect(routerSpy.navigate).toHaveBeenCalledWith(['/simulation-mode']);
//   }));

//   it('should handle null mode and redirect to /home if on a mode page', () => {
//     // Simule qu’on est sur /simulation-mode
//     Object.defineProperty(routerSpy, 'url', { get: () => '/simulation-mode' });

//     fixture.detectChanges();
//     modeSubject.next(null);
//     fixture.detectChanges();

//     expect(routerSpy.navigate).toHaveBeenCalledWith(['/home']);
//   });

//   it('should redirect to /real-mode if mode is REAL', () => {
//     Object.defineProperty(routerSpy, 'url', { get: () => '/home' });

//     fixture.detectChanges();
//     modeSubject.next('REAL');
//     fixture.detectChanges();

//     expect(missionSessionSpy.rehydrateActiveMission).toHaveBeenCalled();
//     expect(routerSpy.navigate).toHaveBeenCalledWith(['/real-mode']);
//   });

//   it('should redirect to /simulation-mode if mode is SIMULATION', () => {
//     Object.defineProperty(routerSpy, 'url', { get: () => '/home' });

//     fixture.detectChanges();
//     modeSubject.next('SIMULATION');
//     fixture.detectChanges();

//     expect(missionSessionSpy.rehydrateActiveMission).toHaveBeenCalled();
//     expect(routerSpy.navigate).toHaveBeenCalledWith(['/simulation-mode']);
//   });
// });
