import { validate } from 'class-validator';
import { UpdateMissionDto } from './update-mission.dto';

describe('UpdateMissionDto', () => {
  it('should validate a correct DTO with all optional fields', async () => {
    const dto = new UpdateMissionDto();
    dto._id = '123456';
    dto.durationSec = 1200;
    dto.robots = ['Atlas-R2', 'Atlas-R3'];
    dto.mode = 'SIMULATION';
    dto.distance = 1500;
    dto.missionName = 'Inspection hangar #12';
    dto.status = 'PENDING';
    dto.logs = [{ message: 'Log test' }];

    const errors = await validate(dto);
    expect(errors.length).toBe(0);
  });

  it('should validate a DTO with only required field _id', async () => {
    const dto = new UpdateMissionDto();
    dto._id = '123456';
    const errors = await validate(dto);
    expect(errors.length).toBe(0);
  });

  it('should fail if _id is missing', async () => {
    const dto = new UpdateMissionDto();
    const errors = await validate(dto);
    expect(errors.some(e => e.property === '_id')).toBe(true);
  });

  it('should fail if durationSec is negative', async () => {
    const dto = new UpdateMissionDto();
    dto._id = '123';
    dto.durationSec = -10;
    const errors = await validate(dto);
    expect(errors.some(e => e.property === 'durationSec')).toBe(true);
  });

  it('should fail if distance is negative', async () => {
    const dto = new UpdateMissionDto();
    dto._id = '123';
    dto.distance = -5;
    const errors = await validate(dto);
    expect(errors.some(e => e.property === 'distance')).toBe(true);
  });

  it('should fail if robots array does not have exactly 2 items', async () => {
    const dto = new UpdateMissionDto();
    dto._id = '123';
    dto.robots = ['Atlas-R2'];
    let errors = await validate(dto);
    expect(errors.some(e => e.property === 'robots')).toBe(true);

    dto.robots = ['Atlas-R1', 'Atlas-R2', 'Atlas-R3'];
    errors = await validate(dto);
    expect(errors.some(e => e.property === 'robots')).toBe(true);
  });

  it('should fail if robots contains non-string elements', async () => {
    const dto = new UpdateMissionDto();
    dto._id = '123';
    // @ts-expect-error
    dto.robots = ['Atlas-R2', 123];
    const errors = await validate(dto);
    expect(errors.some(e => e.property === 'robots')).toBe(true);
  });

  it('should fail if mode is invalid', async () => {
    const dto = new UpdateMissionDto();
    dto._id = '123';
    dto.mode = 'INVALID';
    const errors = await validate(dto);
    expect(errors.some(e => e.property === 'mode')).toBe(true);
  });

  it('should fail if missionName is not a string', async () => {
    const dto = new UpdateMissionDto();
    dto._id = '123';
    // @ts-expect-error
    dto.missionName = 123;
    const errors = await validate(dto);
    expect(errors.some(e => e.property === 'missionName')).toBe(true);
  });

  it('should allow optional fields to be omitted', async () => {
    const dto = new UpdateMissionDto();
    dto._id = '123';
    const errors = await validate(dto);
    expect(errors.length).toBe(0);
  });
});
