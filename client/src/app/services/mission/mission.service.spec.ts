import { TestBed } from '@angular/core/testing';
import { HttpClientTestingModule, HttpTestingController } from '@angular/common/http/testing';
import { MissionService } from './mission.service';
import { environment } from 'src/environments/environment';

describe('MissionService', () => {
  let service: MissionService;
  let httpMock: HttpTestingController;

  beforeEach(() => {
    TestBed.configureTestingModule({
      imports: [HttpClientTestingModule],
      providers: [MissionService]
    });

    service = TestBed.inject(MissionService);
    httpMock = TestBed.inject(HttpTestingController);
  });

  afterEach(() => {
    httpMock.verify();
  });

  it('should be created', () => {
    expect(service).toBeTruthy();
  });

  it('should call /start endpoint on startMission', () => {
    const mockResponse = { success: true };

    service.startMission().subscribe(res => {
      expect(res).toEqual(mockResponse);
    });

    const req = httpMock.expectOne(`${environment.serverUrl}/api/mission/start`);
    expect(req.request.method).toBe('GET');
    req.flush(mockResponse);
  });

  it('should call /stop endpoint on cancelMission', () => {
    const mockResponse = { success: true };

    service.cancelMission().subscribe(res => {
      expect(res).toEqual(mockResponse);
    });

    const req = httpMock.expectOne(`${environment.serverUrl}/api/mission/stop`);
    expect(req.request.method).toBe('GET');
    req.flush(mockResponse);
  });
});
