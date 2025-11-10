import { TestBed } from '@angular/core/testing';
import { SocketService } from './socket.service';

class MockSocket {
  connected = false;
  on = jasmine.createSpy('on');
  once = jasmine.createSpy('once');
  off = jasmine.createSpy('off');
  emit = jasmine.createSpy('emit');
  disconnect = jasmine.createSpy('disconnect');
}

describe('SocketService', () => {
  let service: SocketService;
  let mockSocket: MockSocket;

  beforeEach(() => {
    TestBed.configureTestingModule({
      providers: [SocketService]
    });

    service = TestBed.inject(SocketService);
    mockSocket = new MockSocket();
  });

  it('should be created', () => {
    expect(service).toBeTruthy();
  });

  it('should connect to a namespace', () => {
    (service as any)['socket'] = undefined;
    service.connect('testNamespace');
    expect((service as any)['socket']).toBeDefined();
  });

  it('should not reconnect if socket is already connected', () => {
    mockSocket.connected = true;
    (service as any)['socket'] = mockSocket;
    service.connect('anotherNamespace');
    expect((service as any)['socket']).toBe(mockSocket);
  });

  it('should disconnect socket', () => {
    (service as any)['socket'] = mockSocket;
    service.disconnect();
    expect(mockSocket.disconnect).toHaveBeenCalled();
  });

  it('should call socket.on', () => {
    (service as any)['socket'] = mockSocket;
    const callback = jasmine.createSpy();
    service.on('event', callback);
    expect(mockSocket.on).toHaveBeenCalledWith('event', callback);
  });

  it('should call socket.once', () => {
    (service as any)['socket'] = mockSocket;
    const callback = jasmine.createSpy();
    service.once('event', callback);
    expect(mockSocket.once).toHaveBeenCalledWith('event', callback);
  });

  it('should call socket.off', () => {
    (service as any)['socket'] = mockSocket;
    const callback = jasmine.createSpy();
    service.off('event', callback);
    expect(mockSocket.off).toHaveBeenCalledWith('event', callback);
  });

  it('should emit event with data and callback', () => {
    (service as any)['socket'] = mockSocket;
    const cb = jasmine.createSpy();
    service.send('event', { foo: 'bar' }, cb);
    expect(mockSocket.emit).toHaveBeenCalledWith('event', { foo: 'bar' }, cb);
  });

  it('should emit event without data', () => {
    (service as any)['socket'] = mockSocket;
    service.send('event');
    expect(mockSocket.emit).toHaveBeenCalledWith('event');
  });

  it('should return correct isSocketAlive value', () => {
    (service as any)['socket'] = mockSocket;
    mockSocket.connected = true;
    expect(service.isSocketAlive()).toBeTrue();
    mockSocket.connected = false;
    expect(service.isSocketAlive()).toBeFalse();
  });

  it('should return the socket via getSocket', () => {
    (service as any)['socket'] = mockSocket;
    expect(service.getSocket).toBe(mockSocket as any);
  });
});
