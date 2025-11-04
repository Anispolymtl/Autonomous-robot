import { TestBed } from '@angular/core/testing';
import { Socket } from 'socket.io-client';
import { SocketService } from './socket.service';

describe('SocketService', () => {
    let service: SocketService;
    let mockSocket: jasmine.SpyObj<Socket>;

    beforeEach(() => {
        mockSocket = jasmine.createSpyObj<Socket>('Socket', ['on', 'emit', 'disconnect'], { connected: true, id: '1234' });

        TestBed.configureTestingModule({
            providers: [SocketService],
        });

        service = TestBed.inject(SocketService);
        service.socket = mockSocket;
    });

    describe('Initialization', () => {
        it('should be created', () => {
            expect(service).toBeTruthy();
        });

        it('should return the socket instance', () => {
            expect(service.getSocket).toBe(mockSocket);
        });
    });

    describe('Socket Connection', () => {
        it('should connect to the server', () => {
            spyOn(service, 'connect').and.callThrough();

            service.connect();

            expect(service.connect).toHaveBeenCalled();
            expect(service.socket).toBeDefined();
        });

        it('should disconnect the socket', () => {
            service.disconnect();
            expect(mockSocket.disconnect).toHaveBeenCalled();
        });
    });

    describe('Socket Status', () => {
        it('should check if socket is alive', () => {
            service.socket.connected = true;
            expect(service.isSocketAlive()).toBeTrue();

            service.connect();
            expect(service.isSocketAlive()).toBeFalse();
        });

        it('should return false if socket is undefined', () => {
            service.socket = undefined as unknown as Socket;
            expect(service.isSocketAlive()).toBeFalse();
        });
    });

    describe('Event Handling', () => {
        it('should register event listeners', () => {
            const callback = jasmine.createSpy('callback');
            service.on('test-event', callback);

            expect(mockSocket.on).toHaveBeenCalledWith('test-event', callback);
        });

        it('should send events without callback', () => {
            service.send('test-event', { data: 'test' });

            expect(mockSocket.emit).toHaveBeenCalledWith('test-event', { data: 'test' });
        });

        it('should send events with callback', () => {
            const callback = jasmine.createSpy('callback');
            service.send('test-event', { data: 'test' }, callback);

            expect(mockSocket.emit).toHaveBeenCalledWith('test-event', { data: 'test' }, callback);
        });
    });
});
