import { Component, ViewChild } from '@angular/core';
import { CommonModule } from '@angular/common';
import { Router } from '@angular/router';
import { ReactiveFormsModule, FormGroup } from '@angular/forms';
import { HttpErrorResponse } from '@angular/common/http';
import { IdentifyService } from '@app/services/identify/identify.service';
import { MissionService } from 'src/app/services/mission/mission.service';
import { MapComponent } from '@app/components/map/map.component';
import { RobotStatusComponent } from '@app/components/robot-status/robot-status.component';

@Component({
  selector: 'app-real-page',
  standalone: true,
  imports: [MapComponent, RobotStatusComponent, CommonModule, ReactiveFormsModule],
  templateUrl: './real-mode-page.component.html',
  styleUrls: ['./real-mode-page.component.scss'],
})
export class RealPageComponent {
  form: FormGroup;
  message: string | null = null;

  @ViewChild(RobotStatusComponent)
  robotStatusComponent!: RobotStatusComponent;

  constructor(
    private router: Router,
    private identifyService: IdentifyService,
    private missionService: MissionService
  ) {}

  onIdentify(robotId: number): void {
    this.identifyService.identifyRobot(robotId).subscribe({
      next: (res: any) => {
        this.message = `Réponse du robot ${robotId} : ${res.message}`;
      },
      error: (err: HttpErrorResponse) => {
        this.message = `Erreur lors de l'identification du robot ${robotId} : ${err.message}`;
      },
    });
  }

  startMission(): void {
    this.message = 'Mission demandée.';
    console.log('Mission demandée');
    this.missionService.startMission().subscribe({
      next: (response: any) => {
        console.log('Mission started successfully:', response);
      },
      error: (error: HttpErrorResponse) => {
        console.error('Error starting mission:', error);
      },
    });
  }

  stopMission(): void {
    this.message = 'Mission terminée.';
    console.log('Mission terminée');
    this.missionService.cancelMission().subscribe({
      next: (response: any) => {
        console.log('Mission stopped successfully:', response);
      },
      error: (error: HttpErrorResponse) => {
        console.error('Error stopping mission:', error);
      },
    });
  }

  onSubmit(): void {
    if (this.form.valid) {
      this.message = `Connexion réussie pour l’équipe ${this.form.value.teamName} avec le robot ${this.form.value.robotName}.`;
      console.log('Connexion envoyée', this.form.value);
    } else {
      this.message = 'Veuillez remplir tous les champs.';
    }
  }

  back(): void {
    this.router.navigate(['/home']);
  }
}
