import { Component, OnInit } from '@angular/core';
import { MissionSocketService } from '@app/services/mission-socket/mission-socket.service';

@Component({
  selector: 'app-robot-status',
  standalone: true,
  templateUrl: './robot-status.component.html',
  styleUrls: ['./robot-status.component.scss'],
})
export class RobotStatusComponent implements OnInit {
  robot1Status = 'En attente';
  robot2Status = 'En attente';
  robot1Battery = 21;
  robot2Battery = 67;

  constructor(private missionSocket: MissionSocketService) {}

  ngOnInit() {
    this.missionSocket.getMissionStateObservable().subscribe((state) => {
      if (state === 'running') {
        this.robot1Status = 'En mission';
        this.robot2Status = 'En mission';
      } else {
        this.robot1Status = 'En attente';
        this.robot2Status = 'En attente';
      }
    });
  }
}
