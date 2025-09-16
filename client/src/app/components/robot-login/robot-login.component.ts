import { Component } from '@angular/core';
import { CommonModule } from '@angular/common';
import { FormBuilder, Validators, ReactiveFormsModule } from '@angular/forms';
import { HttpClientModule } from '@angular/common/http';
import { RobotKeyService } from '../../services/robot-key.service';

@Component({
  selector: 'app-robot-login',
  standalone: true,
  imports: [CommonModule, ReactiveFormsModule, HttpClientModule],
  templateUrl: './robot-login.component.html',
  styleUrls: ['./robot-login.component.scss']
})
export class RobotLoginComponent {
  form = this.fb.group({
    robotName: ['', Validators.required],
    teamName: ['', Validators.required],
  });

  loading = false;
  submitted = false;
  generatedKey: string | null = null;
  error: string | null = null;

  constructor(private fb: FormBuilder, private keyService: RobotKeyService) {}

  onSubmit() {
    this.submitted = true;
    this.error = null;

    if (this.form.invalid) return;

    const { robotName, teamName } = this.form.value as { robotName: string; teamName: string };
    this.loading = true;
    this.generatedKey = null;

    this.keyService.getKey(robotName, teamName).subscribe({
      next: (key) => {
        this.generatedKey = key;
        this.loading = false;
      },
      error: (err) => {
        this.error = 'Impossible de générer la clé. Réessayez.';
        console.error(err);
        this.loading = false;
      }
    });
  }
}
