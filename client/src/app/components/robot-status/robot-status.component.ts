import { Component } from '@angular/core';

@Component({
  selector: 'app-robot-status',
  standalone: true,
  templateUrl: './robot-status.component.html',
  styleUrls: ['./robot-status.component.scss'],
})
export class RobotStatusComponent {
  robot1Status = 'En attente';
  robot2Status = 'En attente';
  robot1Battery = 21;
  robot2Battery = 67;
}
