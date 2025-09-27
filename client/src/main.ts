import { provideHttpClient } from '@angular/common/http';
import { enableProdMode } from '@angular/core';
import { bootstrapApplication } from '@angular/platform-browser';
import { provideAnimations } from '@angular/platform-browser/animations';
import { Routes, provideRouter, withHashLocation } from '@angular/router';
import { AppComponent } from '@app/pages/app/app.component';
import { MainPageComponent } from '@app/pages/main-page/main-page.component';
import { environment } from './environments/environment';
import { RobotLoginComponent } from '@app/pages/robot-login-page/robot-login.component';
import { SimulationPageComponent } from '@app/pages/simulation-mode-page/simulation-mode-page.component';
import { RealPageComponent } from '@app/pages/real-mode-page/real-mode-page.component';

if (environment.production) {
    enableProdMode();
}


export const routes: Routes = [
    { path: '', redirectTo: '/home', pathMatch: 'full' },
    { path: 'home', component: MainPageComponent },
    { path: 'robot-login', component: RobotLoginComponent },
    { path: 'simulation-mode', component: SimulationPageComponent },
    { path: 'real-mode', component: RealPageComponent },
    { path: '**', redirectTo: '/home' },
];

bootstrapApplication(AppComponent, {
    providers: [provideHttpClient(), provideRouter(routes, withHashLocation()), provideAnimations()],
})
