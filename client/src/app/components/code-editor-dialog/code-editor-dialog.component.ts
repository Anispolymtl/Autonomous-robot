import { Component, ViewChild } from '@angular/core';
import { MatDialogRef } from '@angular/material/dialog';
import { CodeEditorComponent } from '@app/components/code-editor/code-editor.component';
import { CodeEditorService } from '@app/services/code-editor/code-editor.service';

@Component({
  selector: 'app-code-editor-dialog',
  templateUrl: './code-editor-dialog.component.html',
  styleUrls: ['./code-editor-dialog.component.scss'],
  standalone: true,
  imports: [CodeEditorComponent]
})
export class CodeEditorDialogComponent {
  @ViewChild(CodeEditorComponent) editor!: CodeEditorComponent;
  code: string = '';
  loading = false;
  saving = false;
  errorMessage = '';
  successMessage = '';

  constructor(
    private dialogRef: MatDialogRef<CodeEditorDialogComponent>,
    private codeEditorService: CodeEditorService
  ) {}

  ngOnInit() {
    this.loadCode();
  }

  loadCode() {
    this.loading = true;
    this.codeEditorService.getCode().subscribe({
      next: (res) => {
        this.code = res.code;
        this.editor?.setValue(this.code);
      },
      error: () => {
        this.errorMessage = 'Erreur de chargement du code.';
      },
      complete: () => (this.loading = false)
    });
  }

  save() {
    this.saving = true;
    this.successMessage = '';
    this.errorMessage = '';

    this.codeEditorService.saveCode(this.code).subscribe({
      next: (res) => {
        if (res.success) {
          this.successMessage = res.message || 'Code sauvegardé !';
        } else {
          this.errorMessage = res.message || 'Erreur lors de la sauvegarde.';
        }
      },
      error: (err) => {
        this.errorMessage = err.error?.message || 'Erreur lors de la sauvegarde.';
      },
      complete: () => (this.saving = false)
    });
  }


  close() {
    this.dialogRef.close();
  }
}
