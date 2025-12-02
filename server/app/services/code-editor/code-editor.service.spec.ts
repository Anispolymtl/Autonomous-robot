import { Test, TestingModule } from '@nestjs/testing';
import { CodeEditorService } from './code-editor.service';
import { HttpClient } from '@angular/common/http';
import 'jest';
jest.mock('@angular/common/http');

describe('CodeEditorService', () => {
  let service: CodeEditorService;

  beforeEach(async () => {
    const module: TestingModule = await Test.createTestingModule({
      providers: [
        CodeEditorService,
        { provide: HttpClient, useValue: { get: jest.fn(), post: jest.fn() } },
      ],
    }).compile();

    service = module.get<CodeEditorService>(CodeEditorService);
  });

  it('should be defined', () => {
    expect(service).toBeDefined();
  });
});
