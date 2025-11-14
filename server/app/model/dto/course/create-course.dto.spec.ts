import { validate } from 'class-validator';
import { CreateCourseDto } from './create-course.dto';
import { COURSE_NAME_MAX_LENGTH } from './course.dto.constants';

describe('CreateCourseDto', () => {
  it('should validate a correct DTO', async () => {
    const dto = new CreateCourseDto();
    dto.name = 'Mathématiques';
    dto.teacher = 'Professeur Dupont';
    dto.subjectCode = 'MATH101';
    dto.credits = 3;

    const errors = await validate(dto);
    expect(errors.length).toBe(0);
  });

  it('should fail if name is too long', async () => {
    const dto = new CreateCourseDto();
    dto.name = 'A'.repeat(COURSE_NAME_MAX_LENGTH + 1);
    dto.teacher = 'Professeur Dupont';
    dto.subjectCode = 'MATH101';
    dto.credits = 3;

    const errors = await validate(dto);
    expect(errors.length).toBeGreaterThan(0);
    expect(errors[0].constraints).toHaveProperty('maxLength');
  });

  it('should fail if name is not a string', async () => {
    const dto = new CreateCourseDto();
    // @ts-expect-error
    dto.name = 123;
    dto.teacher = 'Professeur Dupont';
    dto.subjectCode = 'MATH101';
    dto.credits = 3;

    const errors = await validate(dto);
    expect(errors.length).toBeGreaterThan(0);
    expect(errors[0].constraints).toHaveProperty('isString');
  });

  it('should fail if teacher is not a string', async () => {
    const dto = new CreateCourseDto();
    dto.name = 'Mathématiques';
    // @ts-expect-error
    dto.teacher = 456;
    dto.subjectCode = 'MATH101';
    dto.credits = 3;

    const errors = await validate(dto);
    expect(errors.length).toBeGreaterThan(0);
    expect(errors[0].constraints).toHaveProperty('isString');
  });

  it('should fail if subjectCode is not a string', async () => {
    const dto = new CreateCourseDto();
    dto.name = 'Mathématiques';
    dto.teacher = 'Professeur Dupont';
    // @ts-expect-error
    dto.subjectCode = 789;
    dto.credits = 3;

    const errors = await validate(dto);
    expect(errors.length).toBeGreaterThan(0);
    expect(errors[0].constraints).toHaveProperty('isString');
  });

  it('should fail if credits is not a number', async () => {
    const dto = new CreateCourseDto();
    dto.name = 'Mathématiques';
    dto.teacher = 'Professeur Dupont';
    dto.subjectCode = 'MATH101';
    // @ts-expect-error
    dto.credits = '3';

    const errors = await validate(dto);
    expect(errors.length).toBeGreaterThan(0);
    expect(errors[0].constraints).toHaveProperty('isNumber');
  });
});
