import { Component, OnInit, OnDestroy, Input, Output, EventEmitter } from '@angular/core';
import { CommonModule } from '@angular/common';
import { MissionStateService } from '@app/services/state/state.service';
import { Subscription } from 'rxjs';

type RobotId = 'limo1' | 'limo2';

@Component({
  selector: 'app-robot-status',
  standalone: true,
  imports: [CommonModule],
  templateUrl: './robot-status.component.html',
  styleUrls: ['./robot-status.component.scss'],
})
export class RobotStatusComponent implements OnInit, OnDestroy {
  @Input() mode: 'simulation' | 'real' = 'simulation';
  @Output() onReturnToBase = new EventEmitter<void>();
  @Output() onIdentifyRobot = new EventEmitter<RobotId>();

  robot1Status = 'En attente';
  robot2Status = 'En attente';
  robot1MissionStatus = 'Idle';
  robot2MissionStatus = 'Idle';
  explorationStates = ['Frontier Detection', 'Frontier Clustering', 'Frontier Evaluation'];

  robot1Battery = 21;
  robot2Battery = 67;
  robot1Position: { x: number; y: number } | null = null;
  robot2Position: { x: number; y: number } | null = null;

  private sub1!: Subscription;
  private sub2!: Subscription;
  private subMission1!: Subscription;
  private subMission2!: Subscription;
  private subPos1?: Subscription;
  private subPos2?: Subscription;

  constructor(private missionStateService: MissionStateService) {}

  ngOnInit(): void {
    this.missionStateService.connectToSocket();

    this.sub1 = this.missionStateService.getLimo1State$().subscribe((state) => {
      this.robot1Status = state;
    });

    this.sub2 = this.missionStateService.getLimo2State$().subscribe((state) => {
      this.robot2Status = state;
    });

    this.subMission1 = this.missionStateService.getLimo1MissionState$().subscribe((state) => {
      this.robot1MissionStatus = state;
    });

    this.subMission2 = this.missionStateService.getLimo2MissionState$().subscribe((state) => {
      this.robot2MissionStatus = state;
    });

    this.subPos1 = this.missionStateService.getLimo1Position$().subscribe((pos) => {
      this.robot1Position = pos;
    });

    this.subPos2 = this.missionStateService.getLimo2Position$().subscribe((pos) => {
      this.robot2Position = pos;
    });
  }

  ngOnDestroy(): void {
    if (this.sub1) this.sub1.unsubscribe();
    if (this.sub2) this.sub2.unsubscribe();
    if (this.subMission1) this.subMission1.unsubscribe();
    if (this.subMission2) this.subMission2.unsubscribe();
    this.subPos1?.unsubscribe();
    this.subPos2?.unsubscribe();
    this.missionStateService.disconnect();
  }

  returnToBase(): void {
    this.onReturnToBase.emit();
  }

  identifyRobot(robotId: RobotId): void {
    this.onIdentifyRobot.emit(robotId);
  }

  getDisplayStatus(status: string): string {
    const normalized = (status || '').toLowerCase();

    if (!normalized || normalized.startsWith('en attente')) {
      return 'Trajet en attente';
    }

    if (
      normalized.includes('nav') ||
      normalized.includes('waypoint') ||
      normalized.includes('en route') ||
      normalized.includes('moving') ||
      normalized.includes('en cours')
    ) {
      return 'Trajet en cours';
    }

    return status || 'Trajet en cours';
  }
}
