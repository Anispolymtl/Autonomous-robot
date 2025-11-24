import { Controller, Get, Post, Body } from '@nestjs/common';
import { CodeEditorService } from '@app/services/code-editor/code-editor.service';

@Controller('api/code-editor')
export class CodeEditorController {
  constructor(private readonly codeEditorService: CodeEditorService) {}

  @Get('get')
  getCode() {
    return this.codeEditorService.getCode();
  }

  @Post('save')
  saveCode(@Body('code') code: string) {
    return this.codeEditorService.saveCode(code);
  }
}

