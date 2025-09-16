import { Component, OnInit } from '@angular/core';
import { CommonModule } from '@angular/common';
import { ReactiveFormsModule, FormBuilder, FormGroup, Validators } from '@angular/forms';

type ModeType = 'simulation' | 'reel';

@Component({
  selector: 'app-robot-login',
  standalone: true,
  imports: [CommonModule, ReactiveFormsModule],
  templateUrl: './robot-login.component.html',
  styleUrls: ['./robot-login.component.scss'],
})
export class RobotLoginComponent implements OnInit {
  modeForm!: FormGroup;   // contient un booléen: false=simulation, true=reel
  form!: FormGroup;       // formulaire "réel"
  submittedMode = false;
  submitted = false;
  loading = false;
  error: string | null = null;

  selectedMode: ModeType | null = null;

  constructor(private fb: FormBuilder) {}

  ngOnInit(): void {
    this.modeForm = this.fb.group({
      modeToggle: [false], // false = Simulation (par défaut), true = Réel
    });

    this.form = this.fb.group({
      robotName: ['', Validators.required],
      teamName: ['', Validators.required],
    });
  }

 onModeSubmit(): void {
  this.submittedMode = true;
  this.error = null;

  if (this.modeForm.invalid) return;

  this.selectedMode = this.modeForm.value.modeToggle ? 'reel' : 'simulation';

  if (this.selectedMode === 'simulation') {
    this.form.reset();
    this.submitted = false;
    this.startSimulationFlow(); // direct
  }
}
  onSubmit(): void {
    this.submitted = true;
    this.error = null;
    if (this.form.invalid) return;

    this.loading = true;
    const payload = {
      mode: this.selectedMode, // "reel"
      robotName: this.form.value.robotName,
      teamName: this.form.value.teamName,
    };

    try {
      if (!payload.robotName || !payload.teamName) throw new Error('Champs manquants');
      this.loading = false;
      // TODO: navigation ou suite logique
    } catch (e: any) {
      this.loading = false;
      this.error = e?.message ?? 'Une erreur est survenue.';
    }
  }

  private startSimulationFlow(): void {
    // TODO: logique simulation (navigation/init)
  }

  changeMode(): void {
    this.selectedMode = null;
    this.submittedMode = false;
    this.modeForm.setValue({ modeToggle: false });
    this.form.reset();
    this.submitted = false;
    this.error = null;
  }
}
