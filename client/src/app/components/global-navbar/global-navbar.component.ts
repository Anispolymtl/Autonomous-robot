import { AfterViewInit, Component, ElementRef, HostListener, OnDestroy, OnInit, ViewChild } from '@angular/core';
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
export class GlobalNavbarComponent implements OnInit, AfterViewInit, OnDestroy {
  isScrolled = false;
  isHomePage = false;
  currentMode: MissionMode = null;
  private updateFrame: number | null = null;
  private destroy$ = new Subject<void>();

  @ViewChild('navbarRef') navbarRef?: ElementRef<HTMLElement>;

  constructor(private router: Router, private missionModeService: MissionModeService) {}

  ngOnInit(): void {
    this.checkScroll();

    this.router.events
      .pipe(filter((event): event is NavigationEnd => event instanceof NavigationEnd))
      .subscribe((event: NavigationEnd) => {
        this.isHomePage = event.urlAfterRedirects.includes('/home');
        this.scheduleNavbarOffsetUpdate();
      });

    this.missionModeService.mode$
      .pipe(filter((mode): mode is MissionMode => mode !== undefined), takeUntil(this.destroy$))
      .subscribe((mode) => {
        this.currentMode = mode;
        this.scheduleNavbarOffsetUpdate();
      });
  }

  ngAfterViewInit(): void {
    this.scheduleNavbarOffsetUpdate();
  }

  @HostListener('window:scroll', [])
  onWindowScroll(): void {
    this.checkScroll();
  }

  @HostListener('window:resize', [])
  onWindowResize(): void {
    this.scheduleNavbarOffsetUpdate();
  }

  private checkScroll(): void {
    this.isScrolled = window.scrollY > 20;
  }

  private scheduleNavbarOffsetUpdate(): void {
    if (typeof window === 'undefined') return;
    if (this.updateFrame) {
      cancelAnimationFrame(this.updateFrame);
    }
    this.updateFrame = requestAnimationFrame(() => {
      this.updateFrame = null;
      this.updateNavbarOffset();
    });
  }

  private updateNavbarOffset(): void {
    if (typeof document === 'undefined') return;
    const navbarElement = this.navbarRef?.nativeElement;
    if (!navbarElement) return;
    const height = navbarElement.getBoundingClientRect().height;
    document.documentElement.style.setProperty('--navbar-offset', `${height + 16}px`);
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
    if (this.updateFrame) {
      cancelAnimationFrame(this.updateFrame);
    }
  }
}
