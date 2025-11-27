import {
  Component,
  Input,
  Output,
  EventEmitter,
  ViewChild,
  ElementRef,
  AfterViewInit,
  OnDestroy
} from '@angular/core';
import * as monaco from 'monaco-editor';

@Component({
  selector: 'app-code-editor',
  imports: [],
  templateUrl: './code-editor.component.html',
  styleUrl: './code-editor.component.scss'
})
export class CodeEditorComponent implements AfterViewInit, OnDestroy{
  @ViewChild('editorContainer', { static: true }) container!: ElementRef;
  @Input() language: string = 'python';
  @Input() content: string = '';
  @Output() contentChange = new EventEmitter<string>();

  editor?: monaco.editor.IStandaloneCodeEditor;

  ngAfterViewInit(): void {
    this.editor = monaco.editor.create(this.container.nativeElement, {
      value: this.content,
      language: this.language,
      theme: 'vs-dark',
      automaticLayout: true,
      minimap: { enabled: true },
      fontSize: 14
    });

    this.editor.onDidChangeModelContent(() => {
      this.contentChange.emit(this.editor?.getValue() ?? '');
    });
  }

  ngOnDestroy(): void {
    this.editor?.dispose();
  }

  /** Permet à la page parent de mettre à jour le texte (ouvrir fichier) */
  setValue(newText: string) {
    if (this.editor) this.editor.setValue(newText);
  }

}
