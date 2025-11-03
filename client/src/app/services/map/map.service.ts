import { Injectable } from '@angular/core';
import { HttpClient } from '@angular/common/http';
import { Observable, forkJoin, map } from 'rxjs';
import { environment } from 'src/environments/environment';

@Injectable({
  providedIn: 'root'
})
export class MapService {
  constructor(private http: HttpClient) {}

  getGMap(): Observable<{ imageData: ImageData; yaml: any }> {
    const yamlUrl = `${environment.serverUrl}/static/map/map.yaml`;
    const pgmUrl = `${environment.serverUrl}/static/map/map.pgm`;

    return forkJoin({
      yaml: this.http.get(yamlUrl, { responseType: 'text' }).pipe(
        map((yamlText) => this.parseYaml(yamlText))
      ),
      pgm: this.http.get(pgmUrl, { responseType: 'arraybuffer' }),
    }).pipe(
      map(({ yaml, pgm }) => ({
        yaml,
        imageData: this.parsePGM(new Uint8Array(pgm)),
      }))
    );
  }

  private parseYaml(yamlText: string): any {
    const data: any = {};
    for (const line of yamlText.split('\n')) {
      const [key, value] = line.split(':').map((s) => s.trim());
      if (key && value !== undefined) data[key] = value;
    }
    return data;
  }

  private parsePGM(buffer: Uint8Array): ImageData {
    let i = 0;

    const readLine = () => {
      let line = '';
      while (buffer[i] !== 10) {
        line += String.fromCharCode(buffer[i]);
        i++;
      }
      i++;
      return line.trim();
    };

    const magic = readLine();
    if (magic !== 'P5') throw new Error('Only P5 binary PGM supported');

    let line = readLine();
    while (line.startsWith('#')) line = readLine();

    const [width, height] = line.split(' ').map(Number);

    const maxVal = Number(readLine());

    const pixels = buffer.slice(i, i + width * height);

    const canvasData = new Uint8ClampedArray(width * height * 4);

    for (let j = 0; j < width * height; j++) {
      const value = 255 - Math.floor((pixels[j] / maxVal) * 255);
      const idx = j * 4;
      canvasData[idx] = value;
      canvasData[idx + 1] = value;
      canvasData[idx + 2] = value;
      canvasData[idx + 3] = 255;
    }

    return new ImageData(canvasData, width, height);
  }
}
