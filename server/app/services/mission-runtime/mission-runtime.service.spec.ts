import { MissionRuntimeService, MissionCreatePayload } from './mission-runtime.service';

describe('MissionRuntimeService', () => {
  let service: MissionRuntimeService;
  let missionPayload: MissionCreatePayload;

  beforeEach(() => {
    service = new MissionRuntimeService();
    missionPayload = {
      missionName: 'Test Mission',
      robots: ['Atlas-R2', 'Atlas-R3'],
      mode: 'SIMULATION',
      distance: 100,
      durationSec: 60,
    };
  });

  it('should create a mission', () => {
    const mission = service.createMission('socket1', missionPayload);
    expect(mission).toHaveProperty('missionId');
    expect(mission.socketId).toBe('socket1');
    expect(mission.missionName).toBe(missionPayload.missionName);
    expect(service.getActiveMission()).toEqual(mission);
  });

  it('should throw if a mission is already active', () => {
    service.createMission('socket1', missionPayload);
    expect(() => service.createMission('socket2', missionPayload)).toThrow(
      'Une mission est déjà en cours'
    );
  });

  it('should append logs if provided', () => {
    const mission = service.createMission('socket1', missionPayload);
    const logEntry = { message: 'Test log', level: 'INFO' };
    const updated = service.appendLog(mission.missionId, logEntry);

    expect(updated.logs).toHaveLength(1);
    expect(updated.logs?.[0].message).toBe('Test log');
  });

  it('should complete mission', () => {
    const mission = service.createMission('socket1', missionPayload);
    const completed = service.completeMission(mission.missionId);

    expect(completed.status).toBe('COMPLETED');
    expect(service.getActiveMission()).toBeNull();
    expect(service.getCurrentMode()).toBeNull();
  });

  it('should clear mission for socket', () => {
    const mission = service.createMission('socket1', missionPayload);
    service.clearMissionForSocket('socket1');

    expect(service.getActiveMission()).toBeNull();
    expect(service.getCurrentMode()).toBeNull();
  });

  it('should return current mode', () => {
    expect(service.getCurrentMode()).toBeNull();
    service.createMission('socket1', missionPayload);
    expect(service.getCurrentMode()).toBe('SIMULATION');
  });

  it('should throw on unknown mission for update', () => {
    expect(() => service.updateMission('fake-id', { missionName: 'X' })).toThrow('Mission introuvable');
  });

  it('should throw on unknown mission for appendLog', () => {
    expect(() => service.appendLog('fake-id', { message: 'X' })).toThrow('Mission introuvable');
  });

  it('should throw on unknown mission for completeMission', () => {
    expect(() => service.completeMission('fake-id')).toThrow('Mission introuvable');
  });
});
