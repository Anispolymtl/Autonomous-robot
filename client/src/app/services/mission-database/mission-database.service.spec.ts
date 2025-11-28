import { TestBed } from '@angular/core/testing';
import { HttpClientTestingModule, HttpTestingController } from '@angular/common/http/testing';
import { MissionDatabaseService } from './mission-database.service';
import { Mission, MissionStats, CreateMissionDto, UpdateMissionDto } from '@app/interfaces/mission';
import { environment } from 'src/environments/environment';

describe('MissionDatabaseService', () => {
  let service: MissionDatabaseService;
  let httpMock: HttpTestingController;
  const baseUrl = `${environment.serverUrl}/api/missions`;

  const sampleMission: Mission = {
    missionName: 'Test mission',
    mode: 'REAL',
    distance: 10,
    durationSec: 120,
    robots: ['limo1'],
    logs: [],
    status: 'COMPLETED'
  };

  beforeEach(() => {
    TestBed.configureTestingModule({
      imports: [HttpClientTestingModule],
      providers: [MissionDatabaseService]
    });

    service = TestBed.inject(MissionDatabaseService);
    httpMock = TestBed.inject(HttpTestingController);
  });

  afterEach(() => {
    httpMock.verify();
  });

  it('récupère toutes les missions', () => {
    service.getAllMissions().subscribe((missions) => expect(missions).toEqual([sampleMission]));

    const req = httpMock.expectOne(`${baseUrl}/`);
    expect(req.request.method).toBe('GET');
    req.flush([sampleMission]);
  });

  it('ajoute les paramètres de pagination pour les missions', () => {
    service.getAllMissions(6, 6).subscribe();

    const req = httpMock.expectOne(`${baseUrl}/`);
    expect(req.request.method).toBe('GET');
    expect(req.request.params.get('limit')).toBe('6');
    expect(req.request.params.get('skip')).toBe('6');
    req.flush([]);
  });

  it('récupère une mission par identifiant', () => {
    service.getMissionById('abc').subscribe((mission) => expect(mission).toEqual(sampleMission));

    const req = httpMock.expectOne(`${baseUrl}/abc`);
    expect(req.request.method).toBe('GET');
    req.flush(sampleMission);
  });

  it('récupère les missions par robot', () => {
    service.getMissionsByRobot('limo1').subscribe((missions) => expect(missions.length).toBe(1));

    const req = httpMock.expectOne(`${baseUrl}/robot/limo1`);
    expect(req.request.method).toBe('GET');
    req.flush([sampleMission]);
  });

  it('récupère les missions par mode', () => {
    service.getMissionsByMode('SIMULATION').subscribe((missions) => expect(missions).toEqual([sampleMission]));

    const req = httpMock.expectOne(`${baseUrl}/mode/SIMULATION`);
    expect(req.request.method).toBe('GET');
    req.flush([sampleMission]);
  });

  it('ajoute les paramètres de pagination pour les missions par mode', () => {
    service.getMissionsByMode('SIMULATION', 3, 3).subscribe();

    const req = httpMock.expectOne(`${baseUrl}/mode/SIMULATION`);
    expect(req.request.method).toBe('GET');
    expect(req.request.params.get('limit')).toBe('3');
    expect(req.request.params.get('skip')).toBe('3');
    req.flush([]);
  });

  it('récupère les statistiques', () => {
    const stats: MissionStats = {
      total: 5,
      byRobot: { limo1: 5 },
      byMode: { REAL: 3, SIMULATION: 2 },
      totalDistance: 100,
      averageDuration: 42
    };

    service.getMissionStats().subscribe((response) => expect(response).toEqual(stats));

    const req = httpMock.expectOne(`${baseUrl}/stats/overview`);
    expect(req.request.method).toBe('GET');
    req.flush(stats);
  });

  it('crée une mission', () => {
    const newMission: CreateMissionDto = {
      missionName: 'Nouvelle mission',
      mode: 'SIMULATION',
      distance: 0,
      durationSec: 0,
      robots: ['limo1'],
      logs: [],
      status: 'PENDING'
    };

    service.createMission(newMission).subscribe((mission) => expect(mission.missionName).toBe('Nouvelle mission'));

    const req = httpMock.expectOne(`${baseUrl}/`);
    expect(req.request.method).toBe('POST');
    expect(req.request.body).toEqual(newMission);
    req.flush({ ...sampleMission, missionName: 'Nouvelle mission' });
  });

  it('met à jour une mission', () => {
    const updatePayload: UpdateMissionDto = {
      _id: 'abc',
      distance: 42,
      status: 'RUNNING'
    };

    service.updateMission(updatePayload).subscribe((mission) => expect(mission.distance).toBe(42));

    const req = httpMock.expectOne(`${baseUrl}/`);
    expect(req.request.method).toBe('PATCH');
    expect(req.request.body).toEqual(updatePayload);
    req.flush({ ...sampleMission, ...updatePayload });
  });

  it('supprime une mission', () => {
    service.deleteMission('abc').subscribe((res) => expect(res).toEqual({ message: 'deleted' }));

    const req = httpMock.expectOne(`${baseUrl}/abc`);
    expect(req.request.method).toBe('DELETE');
    req.flush({ message: 'deleted' });
  });
});
