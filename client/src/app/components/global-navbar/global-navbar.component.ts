import { Component, HostListener, OnDestroy, OnInit } from '@angular/core';
import { Router, NavigationEnd, RouterLink, RouterLinkActive } from '@angular/router';
import { filter, takeUntil } from 'rxjs/operators';
import { NgIf } from '@angular/common';
import { MissionModeService, MissionMode } from '@app/services/mission-mode.service';
import { Subject } from 'rxjs';

@Component({
  selector: 'app-global-navbar',
  standalone: true,
  imports: [RouterLink, RouterLinkActive, NgIf],
  templateUrl: './global-navbar.component.html',
  styleUrls: ['./global-navbar.component.scss'],
})
export class GlobalNavbarComponent implements OnInit, OnDestroy {
  isScrolled = false;
  isHomePage = false;
  currentMode: MissionMode = null;
  private destroy$ = new Subject<void>();

  constructor(private router: Router, private missionModeService: MissionModeService) {}

  ngOnInit(): void {
    this.checkScroll();

    this.router.events
      .pipe(filter((event): event is NavigationEnd => event instanceof NavigationEnd))
      .subscribe((event: NavigationEnd) => {
        this.isHomePage = event.urlAfterRedirects.includes('/home');
      });
  }

  @HostListener('window:scroll', [])
  onWindowScroll(): void {
    this.checkScroll();
  }

  private checkScroll(): void {
    this.isScrolled = window.scrollY > 20;
    this.missionModeService.mode$
      .pipe(filter((mode): mode is MissionMode => mode !== undefined), takeUntil(this.destroy$))
      .subscribe((mode) => (this.currentMode = mode));
  }

  get showSimulationLink(): boolean {
    if (this.isHomePage) return false;
    return this.currentMode === 'SIMULATION';
  }

  get showRealModeLink(): boolean {
    if (this.isHomePage) return false;
    return this.currentMode === 'REAL';
  }

  get showHomeLink(): boolean {
    return !this.currentMode;
  }

  ngOnDestroy(): void {
    this.destroy$.next();
    this.destroy$.complete();
  }
}
