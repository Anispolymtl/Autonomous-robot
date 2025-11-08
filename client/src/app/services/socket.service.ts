import { Injectable } from '@angular/core';
import { io, Socket } from 'socket.io-client';
import { environment } from 'src/environments/environment';

@Injectable({
    providedIn: 'root',
})
export class SocketService {
    socket: Socket;

    get getSocket(): Socket {
        return this.socket;
    }

    isSocketAlive() {
        return this.socket?.connected ?? false;
    }

    connect(namespace: string) {
        if (this.isSocketAlive()) return;
        const socketUrl = environment.serverUrl;
        this.socket = io(`${socketUrl}/${namespace}`, { transports: ['websocket'], upgrade: false });
    }

    disconnect() {
        if (this.socket) {
            this.socket.disconnect();
        }
    }

        on<T>(event: string, action: (data: T) => void): void {
        if (this.socket) this.socket.on(event, action);
    }

    once(event: string, action: (...args: unknown[]) => void): void {
        if (this.socket) this.socket.once(event, action);
    }

    off(event: string, action: (...args: unknown[]) => void): void {
        if (this.socket) this.socket.off(event, action);
    }

    send<T, R>(event: string, data?: T, callback?: (response: R) => void): void {
        this.socket.emit(event, ...[data, callback].filter((x) => x));
    }
}
