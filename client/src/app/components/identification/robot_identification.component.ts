import { Component } from '@angular/core';
import { IdentifyService } from '../../services/identify.service';

@Component({
    selector: 'app-robot',
    templateUrl: './robot.component.html'
})
export class RobotComponent {
    message = '';

    constructor(private identifyService: IdentifyService) { }

    onIdentify() {
        this.identifyService.identifyRobot().subscribe({
            next: (res) => this.message = res.message,
            error: () => this.message = "Erreur : impossible d'identifier le robot"
        });
    }
}