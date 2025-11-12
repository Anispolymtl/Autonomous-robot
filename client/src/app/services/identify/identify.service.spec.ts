import { TestBed } from '@angular/core/testing';
import { HttpClientTestingModule, HttpTestingController } from '@angular/common/http/testing';
import { IdentifyService } from './identify.service';
import { environment } from 'src/environments/environment';

describe('IdentifyService', () => {
  let service: IdentifyService;
  let httpMock: HttpTestingController;

  beforeEach(() => {
    TestBed.configureTestingModule({
      imports: [HttpClientTestingModule],
      providers: [IdentifyService],
    });

    service = TestBed.inject(IdentifyService);
    httpMock = TestBed.inject(HttpTestingController);
  });

  afterEach(() => {
    httpMock.verify();
  });

  it('should be created', () => {
    expect(service).toBeTruthy();
  });

  it('identifyRobot should call API with correct params', () => {
    const robotId = 42;
    const mockResponse = { id: robotId, name: 'Robo42', status: 'active' };

    service.identifyRobot(robotId).subscribe(res => {
      expect(res).toEqual(mockResponse);
    });

    const req = httpMock.expectOne(req =>
      req.url === `${environment.serverUrl}/api/identify` &&
      req.params.get('id') === robotId.toString()
    );

    expect(req.request.method).toBe('GET');
    req.flush(mockResponse);
  });

  it('identifyRobot should handle error response', () => {
    const robotId = 42;
    const errorMsg = '404 Not Found';

    service.identifyRobot(robotId).subscribe({
      next: () => fail('should have failed'),
      error: (error) => {
        expect(error.statusText).toBe('Not Found');
      },
    });

    const req = httpMock.expectOne(req =>
      req.url === `${environment.serverUrl}/api/identify` &&
      req.params.get('id') === robotId.toString()
    );
    req.flush(errorMsg, { status: 404, statusText: 'Not Found' });
  });
});
