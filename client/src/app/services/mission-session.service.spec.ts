import { TestBed, fakeAsync } from '@angular/core/testing';
import { MissionSessionService } from './mission-session.service';
import { SocketService } from '@app/services/socket.service';
import { MissionModeService } from '@app/services/mission-mode.service';
import { Mission } from '@app/interfaces/mission';

describe('MissionSessionService', () => {
  let service: MissionSessionService;
  let mockSocketService: jasmine.SpyObj<SocketService>;
  let mockMissionModeService: jasmine.SpyObj<MissionModeService>;

  const missionMock: Mission = {
    missionName: 'TestMission',
    robots: ['limo1'],
    mode: 'SIMULATION',
    distance: 10,
    durationSec: 0,
    logs: [],
    status: 'PENDING'
  };

  beforeEach(() => {
    mockSocketService = jasmine.createSpyObj('SocketService', [
      'on',
      'off',
      'send',
      'connect',
      'disconnect',
      'isSocketAlive'
    ]);
    mockMissionModeService = jasmine.createSpyObj('MissionModeService', [
      'setMode',
      'fetchActiveMission'
    ]);

    TestBed.configureTestingModule({
      providers: [
        MissionSessionService,
        { provide: SocketService, useValue: mockSocketService },
        { provide: MissionModeService, useValue: mockMissionModeService }
      ]
    });

    service = TestBed.inject(MissionSessionService);
  });

  it('devrait être créé', () => {
    expect(service).toBeTruthy();
  });

  it('devrait retourner currentMission et hasActiveMission', () => {
    (service as any).missionSnapshot = missionMock;
    (service as any).missionId = '123';
    expect(service.currentMission).toBe(missionMock);
    expect(service.hasActiveMission).toBeTrue();
  });

  it('markMissionStarted ne fait rien si missionId null', () => {
    spyOn<any>(service, 'sendMissionUpdate');
    service.markMissionStarted();
    expect(service['sendMissionUpdate']).not.toHaveBeenCalled();
  });

  it('markMissionStarted envoie mise à jour et log', () => {
    (service as any).missionId = '1';
    spyOn<any>(service, 'sendMissionUpdate');
    spyOn(service as any, 'appendLog');
    service.markMissionStarted();
    expect(service['sendMissionUpdate']).toHaveBeenCalled();
    expect(service['appendLog']).toHaveBeenCalled();
  });

  it('sendMissionUpdate ne fait rien si missionId null', () => {
    service.sendMissionUpdate({ distance: 1 });
    expect(mockSocketService.send).not.toHaveBeenCalled();
  });

  it('sendMissionUpdate calcule durationSec si non défini', () => {
    (service as any).missionId = '1';
    (service as any).missionStartTimestamp = Date.now() - 3000;
    service.sendMissionUpdate({});
    expect(mockSocketService.send).toHaveBeenCalledWith('mission:update', jasmine.any(Object));
  });

  it('updateRobotDistance met à jour correctement', () => {
    (service as any).missionId = '1';
    spyOn(service as any, 'sendMissionUpdate');
    service.updateRobotDistance('limo1', 10);
    expect(service['sendMissionUpdate']).toHaveBeenCalledWith({ distance: 10 });
  });

  it('updateRobotDistance ignore valeurs invalides', () => {
    (service as any).missionId = '1';
    spyOn(service as any, 'sendMissionUpdate');
    service.updateRobotDistance('limo1', -1);
    expect(service['sendMissionUpdate']).not.toHaveBeenCalled();
  });

  it('rehydrateActiveMission ne fait rien si missionId existe', fakeAsync(() => {
    (service as any).missionId = '1';
    service.rehydrateActiveMission();
    expect(mockMissionModeService.fetchActiveMission).not.toHaveBeenCalled();
  }));

  it('appendLog envoie un log normalisé', () => {
    (service as any).missionId = '1';
    service.appendLog({ action: 'test' });
    expect(mockSocketService.send).toHaveBeenCalledWith('mission:add-log', jasmine.any(Object));
  });

  describe('completeMission', () => {
    it('retourne null si missionId absent', async () => {
      const result = await service.completeMission();
      expect(result).toBeNull();
    });
  });

  it('disconnectSocket appelle disconnect si socket vivant', () => {
    mockSocketService.isSocketAlive.and.returnValue(true);
    service.disconnectSocket();
    expect(mockSocketService.disconnect).toHaveBeenCalled();
  });

  describe('ensureSocketConnected', () => {
    it('résout si socket déjà connecté', fakeAsync(async () => {
      mockSocketService.isSocketAlive.and.returnValue(true);
      await service['ensureSocketConnected']();
      expect(mockSocketService.connect).not.toHaveBeenCalled();
    }));
  });

  it('registerMissionListeners ne s’enregistre qu’une seule fois', () => {
    service['registerMissionListeners']();
    service['registerMissionListeners']();
    expect(mockSocketService.on).toHaveBeenCalledTimes(1);
  });


  it('normalizeLogEntry remplit les champs manquants', () => {
    const entry = service['normalizeLogEntry']({});
    expect(entry.category).toBe('Command');
    expect(entry.action).toBe('log');
    expect(entry.robot).toBe('limo1');
    expect(entry.timestamp).toBeTruthy();
  });
});
