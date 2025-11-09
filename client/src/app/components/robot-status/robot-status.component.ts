import { Component, OnInit, OnDestroy } from '@angular/core';
import { MissionStateService } from '@app/services/state.service';
import { Subscription } from 'rxjs';

@Component({
  selector: 'app-robot-status',
  standalone: true,
  templateUrl: './robot-status.component.html',
  styleUrls: ['./robot-status.component.scss'],
})
export class RobotStatusComponent implements OnInit, OnDestroy {
  robot1Status = 'En attente';
  robot2Status = 'En attente';
  robot1Battery = 21;
  robot2Battery = 67;

  private sub1!: Subscription;
  private sub2!: Subscription;

  constructor(private missionStateService: MissionStateService) {}

  ngOnInit(): void {
    // ✅ Connexion aux namespaces ROS2 (sockets)
    this.missionStateService.connectToSocket();
    
    // ✅ Écoute en temps réel des états
    this.sub1 = this.missionStateService.getLimo1State$().subscribe((state) => {
      this.robot1Status = state;
    });

    this.sub2 = this.missionStateService.getLimo2State$().subscribe((state) => {
      this.robot2Status = state;
    });
  }

  ngOnDestroy(): void {
    // ✅ Nettoyage
    if (this.sub1) this.sub1.unsubscribe();
    if (this.sub2) this.sub2.unsubscribe();
    this.missionStateService.disconnect();
  }
}
