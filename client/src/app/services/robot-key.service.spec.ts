import { TestBed } from '@angular/core/testing';

import { RobotKeyService } from './robot-key.service';

describe('RobotKeyService', () => {
  let service: RobotKeyService;

  beforeEach(() => {
    TestBed.configureTestingModule({});
    service = TestBed.inject(RobotKeyService);
  });

  it('should be created', () => {
    expect(service).toBeTruthy();
  });
});
