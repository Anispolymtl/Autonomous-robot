import { validate } from 'class-validator';
import { UpdateCourseDto } from './update-course.dto';
import { COURSE_NAME_MAX_LENGTH } from './course.dto.constants';

describe('UpdateCourseDto', () => {
  it('should validate a correct DTO with all optional fields', async () => {
    const dto = new UpdateCourseDto();
    dto.name = 'Physique';
    dto.teacher = 'Professeur Einstein';
    dto.subjectCode = 'PHYS101';
    dto.credits = 4;

    const errors = await validate(dto);
    expect(errors.length).toBe(0);
  });

  it('should validate a DTO with only required field', async () => {
    const dto = new UpdateCourseDto();
    dto.subjectCode = 'CHEM101';

    const errors = await validate(dto);
    expect(errors.length).toBe(0);
  });

  it('should fail if name is too long', async () => {
    const dto = new UpdateCourseDto();
    dto.name = 'A'.repeat(COURSE_NAME_MAX_LENGTH + 1);
    dto.subjectCode = 'BIO101';

    const errors = await validate(dto);
    expect(errors.length).toBeGreaterThan(0);
    expect(errors[0].constraints).toHaveProperty('maxLength');
  });

  it('should fail if name is not a string', async () => {
    const dto = new UpdateCourseDto();
    // @ts-expect-error
    dto.name = 123;
    dto.subjectCode = 'BIO101';

    const errors = await validate(dto);
    expect(errors.length).toBeGreaterThan(0);
    expect(errors[0].constraints).toHaveProperty('isString');
  });

  it('should fail if teacher is not a string', async () => {
    const dto = new UpdateCourseDto();
    dto.subjectCode = 'CHEM101';
    // @ts-expect-error
    dto.teacher = 456;

    const errors = await validate(dto);
    expect(errors.length).toBeGreaterThan(0);
    expect(errors[0].constraints).toHaveProperty('isString');
  });

  it('should fail if subjectCode is not a string', async () => {
    const dto = new UpdateCourseDto();
    // @ts-expect-error
    dto.subjectCode = 789;

    const errors = await validate(dto);
    expect(errors.length).toBeGreaterThan(0);
    expect(errors[0].constraints).toHaveProperty('isString');
  });

  it('should fail if credits is not a number', async () => {
    const dto = new UpdateCourseDto();
    dto.subjectCode = 'MATH101';
    // @ts-expect-error
    dto.credits = '3';

    const errors = await validate(dto);
    expect(errors.length).toBeGreaterThan(0);
    expect(errors[0].constraints).toHaveProperty('isNumber');
  });
});
