import { Component } from '@angular/core';
import { RobotLoginComponent } from '@app/pages/robot-login-page/robot-login.component';


@Component({
    selector: 'app-main-page',
    templateUrl: './main-page.component.html',
    styleUrls: ['./main-page.component.scss'],
    imports: [RobotLoginComponent],
})
export class MainPageComponent {
}
