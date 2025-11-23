import { CommonModule } from '@angular/common';
import { Component, Inject } from '@angular/core';
import { MAT_DIALOG_DATA, MatDialogRef } from '@angular/material/dialog';
import { Mission } from '@app/interfaces/mission';

@Component({
    selector: 'app-mission-maps-dialog',
    standalone: true,
    imports: [CommonModule],
    templateUrl: './mission-maps-dialog.component.html',
    styleUrls: ['./mission-maps-dialog.component.scss']
})
export class MissionMapsDialogComponent {
    constructor(
        private dialogRef: MatDialogRef<MissionMapsDialogComponent>,
        @Inject(MAT_DIALOG_DATA) public data: { mission: Mission }
    ) {}

    close(): void {
        this.dialogRef.close();
    }
}
