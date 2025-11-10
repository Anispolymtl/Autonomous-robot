import { TestBed, fakeAsync, tick } from '@angular/core/testing';
import { RobotKeyService } from './robot-key.service';
import { HttpClient } from '@angular/common/http';
import { of } from 'rxjs';

describe('RobotKeyService', () => {
  let service: RobotKeyService;
  let httpClientSpy: jasmine.SpyObj<HttpClient>;

  beforeEach(() => {
    httpClientSpy = jasmine.createSpyObj('HttpClient', ['post']);

    TestBed.configureTestingModule({
      providers: [
        RobotKeyService,
        { provide: HttpClient, useValue: httpClientSpy }
      ]
    });

    service = TestBed.inject(RobotKeyService);
  });

  it('should be created', () => {
    expect(service).toBeTruthy();
  });

  it('should return a key string with robotName and teamName (mock mode)', fakeAsync(() => {
    let keyValue: string | undefined;
    service.getKey('RobotA', 'TeamX').subscribe(k => (keyValue = k));

    tick(500);

    expect(keyValue).toBeDefined();
    expect(keyValue).toMatch(/^RobotA-TeamX-/);
  }));

  it('should call HttpClient.post when USE_HTTP = true', () => {
    (service as any).USE_HTTP = true;

    const fakeKey = 'ROBOT-TIME-KEY';
    httpClientSpy.post.and.returnValue(of({ key: fakeKey }));

    let result: string | undefined;
    service.getKey('RobotB', 'TeamY').subscribe(k => (result = k));

    expect(httpClientSpy.post).toHaveBeenCalledOnceWith(
      '/api/robot-key',
      { robotName: 'RobotB', teamName: 'TeamY' }
    );
    expect(result).toBe(fakeKey);
  });
});
