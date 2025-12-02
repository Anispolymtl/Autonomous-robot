import { CommonModule } from '@angular/common';
import { Component, Inject } from '@angular/core';
import { MAT_DIALOG_DATA, MatDialogRef } from '@angular/material/dialog';

export interface ConfirmationDialogData {
  title?: string;
  message?: string;
  consequences?: string[];
  confirmText?: string;
  cancelText?: string;
  tone?: 'danger' | 'info';
}

@Component({
  selector: 'app-confirmation-dialog',
  standalone: true,
  imports: [CommonModule],
  templateUrl: './confirmation-dialog.component.html',
  styleUrls: ['./confirmation-dialog.component.scss'],
})
export class ConfirmationDialogComponent {
  constructor(
    private dialogRef: MatDialogRef<ConfirmationDialogComponent>,
    @Inject(MAT_DIALOG_DATA) public data: ConfirmationDialogData
  ) {
    this.data = {
      title: 'Confirmer l’action',
      message: 'Êtes-vous sûr de vouloir continuer ?',
      consequences: [],
      confirmText: 'Confirmer',
      cancelText: 'Annuler',
      tone: 'info',
      ...data,
    };
  }

  close(result: boolean): void {
    this.dialogRef.close(result);
  }
}
