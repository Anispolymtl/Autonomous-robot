import { TestBed, fakeAsync, tick } from '@angular/core/testing';
import { of } from 'rxjs';
import { HttpClientTestingModule, HttpTestingController } from '@angular/common/http/testing';
import { MissionModeService, MissionMode, ActiveMissionResponse } from './mission-mode.service';

describe('MissionModeService', () => {
  let service: MissionModeService;
  let httpMock: HttpTestingController;

  beforeEach(() => {
    TestBed.configureTestingModule({
      imports: [HttpClientTestingModule],
      providers: [MissionModeService]
    });

    service = TestBed.inject(MissionModeService);
    httpMock = TestBed.inject(HttpTestingController);
  });

  afterEach(() => {
    service.stopPolling();
    httpMock.verify();
  });

  it('should be created', () => {
    expect(service).toBeTruthy();
  });

  it('currentMode should reflect BehaviorSubject value', () => {
    expect(service.currentMode).toBeUndefined();
    service.setMode('REAL');
    expect(service.currentMode).toBe('REAL');
  });

  it('refreshMode should call API and update modeSubject', () => {
    const mockResponse = { mode: 'SIMULATION' } as { mode: MissionMode };
    service.refreshMode().subscribe(mode => {
      expect(mode).toBe('SIMULATION');
      expect(service.currentMode).toBe('SIMULATION');
    });

    const req = httpMock.expectOne(`${service['missionApi']}/current-mode`);
    expect(req.request.method).toBe('GET');
    req.flush(mockResponse);
  });

  it('ensureModeLoaded should resolve immediately if mode already loaded', async () => {
    service.setMode('REAL');
    const result = await service.ensureModeLoaded();
    expect(result).toBe('REAL');
  });

  it('ensureModeLoaded should fetch mode if undefined', async () => {
    const mockResponse = { mode: 'SIMULATION' } as { mode: MissionMode };
    const promise = service.ensureModeLoaded();

    const req = httpMock.expectOne(`${service['missionApi']}/current-mode`);
    req.flush(mockResponse);

    const result = await promise;
    expect(result).toBe('SIMULATION');
  });

  it('ensureModeLoaded should return null on API error', async () => {
    const promise = service.ensureModeLoaded();

    const req = httpMock.expectOne(`${service['missionApi']}/current-mode`);
    req.error(new ErrorEvent('Network error'));

    const result = await promise;
    expect(result).toBeNull();
  });

  it('fetchActiveMission should call API and return mission', () => {
    const mockMission: ActiveMissionResponse = {
      missionId: '123',
      durationSec: 10,
      robots: ['r1'],
      mode: 'REAL',
      distance: 5,
      missionName: 'Test',
    };

    service.fetchActiveMission().subscribe(res => {
      expect(res).toEqual(mockMission);
    });

    const req = httpMock.expectOne(`${service['missionApi']}/active`);
    expect(req.request.method).toBe('GET');
    req.flush(mockMission);
  });

  it('fetchActiveMission should return null on API error', () => {
    service.fetchActiveMission().subscribe(res => {
      expect(res).toBeNull();
    });

    const req = httpMock.expectOne(`${service['missionApi']}/active`);
    req.error(new ErrorEvent('Network error'));
  });

  it('startPolling should call refreshMode periodically', fakeAsync(() => {
    spyOn(service, 'refreshMode').and.returnValue(of('REAL'));

    service.startPolling(10);
    expect(service['pollingHandle']).toBeDefined();

    tick(15);
    expect(service.refreshMode).toHaveBeenCalled();

    service.stopPolling();
    expect(service['pollingHandle']).toBeNull();
  }));

  it('stopPolling should clear interval', () => {
    service['pollingHandle'] = setInterval(() => {}, 1000);
    service.stopPolling();
    expect(service['pollingHandle']).toBeNull();
  });
});
