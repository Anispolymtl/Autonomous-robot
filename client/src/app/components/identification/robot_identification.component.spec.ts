// import { ComponentFixture, TestBed } from '@angular/core/testing';
// import { of, throwError } from 'rxjs';
// import { RobotComponent } from '@app/components/identification/robot_identification.component';
// import { IdentifyService } from '@app/services/identify.service';

// describe('RobotComponent', () => {
//   let component: RobotComponent;
//   let fixture: ComponentFixture<RobotComponent>;
//   let identifyServiceSpy: jasmine.SpyObj<IdentifyService>;

//   beforeEach(async () => {
//     identifyServiceSpy = jasmine.createSpyObj('IdentifyService', ['identifyRobot']);

//     await TestBed.configureTestingModule({
//       declarations: [RobotComponent],
//       providers: [{ provide: IdentifyService, useValue: identifyServiceSpy }]
//     }).compileComponents();

//     fixture = TestBed.createComponent(RobotComponent);
//     component = fixture.componentInstance;
//     fixture.detectChanges();
//   });

//   it('should call identifyRobot and update message on success', () => {
//     identifyServiceSpy.identifyRobot.and.returnValue(of({ message: 'Robot identifié avec succès' }));

//     component.onIdentify();

//     expect(identifyServiceSpy.identifyRobot).toHaveBeenCalled();
//     expect(component.message).toBe('Robot identifié avec succès');
//   });

//   it('should set error message when identifyRobot fails', () => {
//     identifyServiceSpy.identifyRobot.and.returnValue(throwError(() => new Error('Erreur serveur')));

//     component.onIdentify();

//     expect(identifyServiceSpy.identifyRobot).toHaveBeenCalled();
//     expect(component.message).toBe("Erreur : impossible d'identifier le robot");
//   });
// });
