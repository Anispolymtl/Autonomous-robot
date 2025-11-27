import { Component, ViewChild, AfterViewInit } from '@angular/core';
import { MatDialogRef } from '@angular/material/dialog';
import { MatIconModule } from '@angular/material/icon';
import { MatProgressBarModule } from '@angular/material/progress-bar';
import { MatSnackBar, MatSnackBarModule } from '@angular/material/snack-bar';
import { CodeEditorComponent } from '@app/components/code-editor/code-editor.component';
import { CodeEditorService } from '@app/services/code-editor/code-editor.service';

@Component({
  selector: 'app-code-editor-dialog',
  templateUrl: './code-editor-dialog.component.html',
  styleUrls: ['./code-editor-dialog.component.scss'],
  standalone: true,
  imports: [
    CodeEditorComponent,
    MatIconModule,
    MatProgressBarModule,
    MatSnackBarModule
  ]
})
export class CodeEditorDialogComponent implements AfterViewInit {

  @ViewChild(CodeEditorComponent) editor!: CodeEditorComponent;

  code: string = '';
  loading = false;
  saving = false;

  editorReady = false;

  constructor(
    private dialogRef: MatDialogRef<CodeEditorDialogComponent>,
    private codeEditorService: CodeEditorService,
    private snackBar: MatSnackBar
  ) {}

  ngOnInit() {
    this.loadCode();
  }

  ngAfterViewInit() {
    this.editorReady = true;

    if (this.code) {
      this.editor.setValue(this.code);
    }
  }

  // --- SNACKBAR HELPERS ---
  private showSuccess(message: string) {
    this.snackBar.open(message, 'OK', {
      duration: 2500,
      horizontalPosition: 'center',
      verticalPosition: 'top',
      panelClass: ['snackbar-success']
    });
  }

  private showError(message: string) {
    this.snackBar.open(message, 'Fermer', {
      duration: 4000,
      horizontalPosition: 'center',
      verticalPosition: 'top',
      panelClass: ['snackbar-error']
    });
  }

  loadCode() {
    this.loading = true;

    this.codeEditorService.getCode().subscribe({
      next: (res) => {
        this.code = res.code;

        if (this.editorReady) {
          this.editor.setValue(this.code);
        }
      },
      error: () => {
        this.showError('Erreur de chargement du code.');
      },
      complete: () => (this.loading = false)
    });
  }

  save() {
    this.saving = true;

    this.codeEditorService.saveCode(this.code).subscribe({
      next: (res) => {
        if (res.success) {
          this.showSuccess(res.message || 'Code sauvegardé !');

          // ⏳ Fermeture automatique après feedback
          setTimeout(() => this.dialogRef.close(true), 300);
        } else {
          this.showError(res.message || 'Erreur lors de la sauvegarde.');
        }
      },
      error: (err) => {
        this.showError(err.error?.message || 'Erreur lors de la sauvegarde.');
      },
      complete: () => (this.saving = false)
    });
  }

  reset() {
    this.codeEditorService.getCode().subscribe({
      next: (res) => {
        this.code = res.code;
        if (this.editorReady) this.editor.setValue(this.code);

        this.showSuccess('Code réinitialisé.');
      },
      error: () => {
        this.showError('Erreur en rechargeant le code.');
      }
    });
  }

  close() {
    this.dialogRef.close();
  }
}
