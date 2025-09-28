import { Component, OnInit } from '@angular/core';
import { CommonModule } from '@angular/common';
import { FormBuilder, FormGroup, ReactiveFormsModule } from '@angular/forms';
import { Router } from '@angular/router';

type ModeType = 'simulation' | 'real';

@Component({
  selector: 'app-robot-login',
  standalone: true,
  imports: [CommonModule, ReactiveFormsModule],
  templateUrl: './robot-login.component.html',
  styleUrls: ['./robot-login.component.scss'],
})
export class RobotLoginComponent implements OnInit {
  modeForm!: FormGroup;
  selectedMode: ModeType | null = null;

  constructor(private fb: FormBuilder, private router: Router) { }

  ngOnInit(): void {
    this.modeForm = this.fb.group({
      modeToggle: [false], // false = Simulation, true = Réel
    });
  }

  onModeSubmit(): void {
    this.selectedMode = this.modeForm.value.modeToggle ? 'real' : 'simulation';
    if (this.selectedMode === 'simulation') {
      this.router.navigate(['/simulation-mode']);
    } else {
      this.router.navigate(['/real-mode']);
    }
  }
}