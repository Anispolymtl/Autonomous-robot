import { ComponentFixture, TestBed } from '@angular/core/testing';

import { CodeEditorPageComponent } from './code-editor-page.component';

describe('CodeEditorPageComponent', () => {
  let component: CodeEditorPageComponent;
  let fixture: ComponentFixture<CodeEditorPageComponent>;

  beforeEach(async () => {
    await TestBed.configureTestingModule({
      imports: [CodeEditorPageComponent]
    })
    .compileComponents();

    fixture = TestBed.createComponent(CodeEditorPageComponent);
    component = fixture.componentInstance;
    fixture.detectChanges();
  });

  it('should create', () => {
    expect(component).toBeTruthy();
  });
});
