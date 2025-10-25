import { Component } from '@angular/core';
import { RouterOutlet } from '@angular/router';
import { GlobalNavbarComponent } from '@app/components/global-navbar/global-navbar.component';

@Component({
    selector: 'app-root',
    templateUrl: './app.component.html',
    styleUrls: ['./app.component.scss'],
    imports: [RouterOutlet, GlobalNavbarComponent],
})
export class AppComponent {}
