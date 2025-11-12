import { TestBed } from '@angular/core/testing';
import { HttpClientTestingModule, HttpTestingController } from '@angular/common/http/testing';
import { MissionDatabaseService } from './mission-database.service';
import { Mission, UpdateMissionDto, MissionLogEntry } from '@app/interfaces/mission';

describe('MissionDatabaseService', () => {
  let service: MissionDatabaseService;
  let httpMock: HttpTestingController;

  beforeEach(() => {
    TestBed.configureTestingModule({
      imports: [HttpClientTestingModule],
      providers: [MissionDatabaseService],
    });

    service = TestBed.inject(MissionDatabaseService);
    httpMock = TestBed.inject(HttpTestingController);
  });

  afterEach(() => {
    httpMock.verify();
  });

  it('should update a mission', () => {
    const updatedMission: UpdateMissionDto = {
      _id: '123',
      durationSec: 120,
      distance: 500,
      robots: ['r1', 'r2'],
      mode: 'REAL',
      missionName: 'Updated Mission',
      logs: [] as MissionLogEntry[],
      status: 'IN_PROGRESS',
    };

    const mockResponse: Mission = {
      _id: updatedMission._id,
      durationSec: updatedMission.durationSec!,
      distance: updatedMission.distance!,
      robots: updatedMission.robots!,
      mode: updatedMission.mode!,
      missionName: updatedMission.missionName!,
      logs: updatedMission.logs,
      status: updatedMission.status,
    };

    service.updateMission(updatedMission).subscribe((res) => {
      expect(res).toEqual(mockResponse);
    });

    const req = httpMock.expectOne(`${service['missionsUrl']}/`);
    expect(req.request.method).toBe('PATCH');
    req.flush(mockResponse);
  });
});
