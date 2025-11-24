import { Injectable } from '@angular/core';
import { HttpClient } from '@angular/common/http';
import { Observable } from 'rxjs';
import { environment } from 'src/environments/environment';

export interface GetCodeResponse {
  code: string;
  success: boolean;
  message: string;
}

export interface SaveCodeResponse {
  success: boolean;
  message: string;
}

@Injectable({
  providedIn: 'root'
})
export class CodeEditorService {

  // EXACTEMENT comme IdentifyService
  private apiUrl = `${environment.serverUrl}/api/code-editor`;

  constructor(private http: HttpClient) { }

  /**
   * Récupérer le fichier mission_logic.py
   */
  getCode(): Observable<GetCodeResponse> {
    return this.http.get<GetCodeResponse>(`${this.apiUrl}/get`);
  }

  /**
   * Sauvegarder un nouveau code dans mission_logic.py
   */
  saveCode(code: string): Observable<SaveCodeResponse> {
    return this.http.post<SaveCodeResponse>(`${this.apiUrl}/save`, { code });
  }
}
