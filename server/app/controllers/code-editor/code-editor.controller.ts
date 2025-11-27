import { Controller, Get, Post, Body } from '@nestjs/common';
import { CodeEditorService } from '@app/services/code-editor/code-editor.service';

@Controller('/code-editor')
export class CodeEditorController {
  constructor(private readonly codeEditorService: CodeEditorService) {}

  @Get('get')
  async getCode() {
    return await this.codeEditorService.getCode();
  }

  @Post('save')
  async saveCode(@Body('code') code: string) {
    return await this.codeEditorService.saveCode(code);
  }

  @Post('restore-default')
  async restoreDefaultMission() {
    console.log('Restore Default Mission')
    return await this.codeEditorService.restoreMission();
  }
}

