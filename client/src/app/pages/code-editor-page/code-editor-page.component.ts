import { Component, ViewChild } from '@angular/core';
import { CodeEditorService } from '@app/services/code-editor/code-editor.service';
import { CodeEditorComponent } from '@app/components/code-editor/code-editor.component';

@Component({
  selector: 'app-code-editor-page',
  standalone: true,
  imports: [CodeEditorComponent],
  templateUrl: './code-editor-page.component.html',
  styleUrls: ['./code-editor-page.component.scss']
})
export class CodeEditorPageComponent {

  code: string = '';
  loading: boolean = false;
  saving: boolean = false;
  errorMessage: string = '';
  successMessage: string = '';

  @ViewChild(CodeEditorComponent) editor!: CodeEditorComponent;

  constructor(private codeEditorService: CodeEditorService) {}

  ngOnInit() {
    this.loadCode();
  }

  /** Chargement initial du code */
  loadCode() {
    this.loading = true;
    this.errorMessage = '';
    this.successMessage = '';

    this.codeEditorService.getCode().subscribe({
      next: (res) => {
        if (res.success) {
          this.code = res.code;

          if (this.editor) {
            this.editor.setValue(res.code);
          }
        } else {
          this.errorMessage = res.message;
        }
      },
      error: () => {
        this.errorMessage = 'Erreur : impossible de contacter le backend';
      },
      complete: () => {
        this.loading = false;
      }
    });
  }

  /** Émission des changements */
  onCodeChanged(text: string) {
    this.code = text;
  }

  /** Sauvegarde du code */
  saveCode() {
    this.saving = true;
    this.errorMessage = '';
    this.successMessage = '';

    this.codeEditorService.saveCode(this.code).subscribe({
      next: (res) => {
        if (res.success) {
          this.successMessage = res.message || 'Code sauvegardé avec succès';
        } else {
          this.errorMessage = res.message || 'Erreur lors de la sauvegarde';
        }
      },
      error: () => {
        this.errorMessage = 'Échec : impossible de contacter le backend';
      },
      complete: () => {
        this.saving = false;
      }
    });
  }
}
