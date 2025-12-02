declare module '@angular/common/http' {
  export class HttpClient {
    get<T = any>(...args: any[]): any;
    post<T = any>(...args: any[]): any;
  }
}
