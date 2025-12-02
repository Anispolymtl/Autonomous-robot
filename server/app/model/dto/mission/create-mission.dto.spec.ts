import { validate } from 'class-validator';
import { CreateMissionDto } from './create-mission.dto';

describe('CreateMissionDto', () => {
  it('should validate a correct DTO', async () => {
    const dto = new CreateMissionDto();
    dto.durationSec = 1200;
    dto.robots = ['Atlas-R2', 'Atlas-R3'];
    dto.mode = 'SIMULATION';
    dto.distance = 1500;
    dto.missionName = 'Inspection hangar #12';
    dto.status = 'PENDING';
    dto.logs = [{
      timestamp: new Date().toISOString(),
      robot: 'Atlas-R2',
      category: 'Command',
      action: 'log',
      details: {}
    }];

    const errors = await validate(dto);
    expect(errors.length).toBe(0);
  });

  it('should fail if durationSec is negative', async () => {
    const dto = new CreateMissionDto();
    dto.durationSec = -1;
    dto.robots = ['Atlas-R2', 'Atlas-R3'];
    dto.mode = 'SIMULATION';
    dto.distance = 1500;
    dto.missionName = 'Test mission';

    const errors = await validate(dto);
    expect(errors.some(e => e.property === 'durationSec')).toBe(true);
  });

  it('should fail if robots array does not have exactly 2 items', async () => {
    const dto = new CreateMissionDto();
    dto.durationSec = 100;
    dto.robots = ['Atlas-R2'];
    dto.mode = 'REAL';
    dto.distance = 500;
    dto.missionName = 'Test mission';

    const errors = await validate(dto);
    expect(errors.some(e => e.property === 'robots')).toBe(true);

    dto.robots = ['Atlas-R1', 'Atlas-R2', 'Atlas-R3'];
    const errors2 = await validate(dto);
    expect(errors2.some(e => e.property === 'robots')).toBe(true);
  });

  it('should fail if robots contains non-string elements', async () => {
    const dto = new CreateMissionDto();
    // @ts-expect-error
    dto.robots = ['Atlas-R2', 123];
    dto.durationSec = 100;
    dto.mode = 'SIMULATION';
    dto.distance = 200;
    dto.missionName = 'Test mission';

    const errors = await validate(dto);
    expect(errors.some(e => e.property === 'robots')).toBe(true);
  });

  it('should fail if mode is invalid', async () => {
    const dto = new CreateMissionDto();
    dto.durationSec = 100;
    dto.robots = ['Atlas-R2', 'Atlas-R3'];
    dto.mode = 'INVALID';
    dto.distance = 100;
    dto.missionName = 'Test mission';

    const errors = await validate(dto);
    expect(errors.some(e => e.property === 'mode')).toBe(true);
  });

  it('should fail if distance is negative', async () => {
    const dto = new CreateMissionDto();
    dto.durationSec = 100;
    dto.robots = ['Atlas-R2', 'Atlas-R3'];
    dto.mode = 'REAL';
    dto.distance = -10;
    dto.missionName = 'Test mission';

    const errors = await validate(dto);
    expect(errors.some(e => e.property === 'distance')).toBe(true);
  });

  it('should fail if missionName is not a string', async () => {
    const dto = new CreateMissionDto();
    dto.durationSec = 100;
    dto.robots = ['Atlas-R2', 'Atlas-R3'];
    dto.mode = 'SIMULATION';
    dto.distance = 100;
    // @ts-expect-error
    dto.missionName = 123;

    const errors = await validate(dto);
    expect(errors.some(e => e.property === 'missionName')).toBe(true);
  });

  it('should allow optional fields to be omitted', async () => {
    const dto = new CreateMissionDto();
    dto.durationSec = 100;
    dto.robots = ['Atlas-R2', 'Atlas-R3'];
    dto.mode = 'REAL';
    dto.distance = 200;
    dto.missionName = 'Test mission';

    const errors = await validate(dto);
    expect(errors.length).toBe(0);
  });
});
