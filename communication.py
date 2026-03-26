import socket
import struct
import threading
from queue import Empty, Queue

class Server:
    def __init__(self, port):
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.socket.bind(("0.0.0.0", port))
        self.socket.listen(1)
        self.socket.settimeout(0.2)
        self.connection = None
        self.running = False
        self.send_queue = Queue()
        self.accept_thread = None
        self.send_thread = None

    def start(self):
        self.running = True
        self.accept_thread = threading.Thread(target=self._accept_loop, daemon=True)
        self.send_thread = threading.Thread(target=self._send_loop, daemon=True)
        self.accept_thread.start()
        self.send_thread.start()

    def broadcast_coordinates(self, first_value, second_value):
        packet = struct.pack("!ii", first_value, second_value)
        self.send_queue.put_nowait(packet)

    def stop(self):
        self.running = False

        if self.connection is not None:
            self.connection.close()
            self.connection = None

        self.socket.close()

        if self.accept_thread is not None:
            self.accept_thread.join(timeout=0.5)

        if self.send_thread is not None:
            self.send_thread.join(timeout=0.5)

    def _accept_loop(self):
        while self.running and self.connection is None:
            try:
                self.connection, _ = self.socket.accept()
            except TimeoutError:
                continue
            except OSError:
                return

    def _send_loop(self):
        while self.running:
            try:
                packet = self.send_queue.get(timeout=0.2)
            except Empty:
                continue

            while self.running and self.connection is None:
                threading.Event().wait(0.05)

            if not self.running or self.connection is None:
                return

            try:
                self.connection.sendall(packet)
            except OSError:
                self.connection = None


class Client:
    def __init__(self, host, port):
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.host = host
        self.port = port
        self.running = False
        self.receiver_thread = None
        self.last_coordinates = None
        self.last_coordinates_lock = threading.Lock()

    def start(self):
        self.socket.connect((self.host, self.port))
        self.socket.settimeout(0.2)
        self.running = True
        self.receiver_thread = threading.Thread(target=self._receive_loop, daemon=True)
        self.receiver_thread.start()

    def receive_coordinates(self):
        with self.last_coordinates_lock:
            return self.last_coordinates

    def stop(self):
        self.running = False
        self.socket.close()

        if self.receiver_thread is not None:
            self.receiver_thread.join(timeout=0.5)

    def _receive_loop(self):
        expected_bytes = struct.calcsize("!ii")
        buffer = b""

        while self.running:
            try:
                chunk = self.socket.recv(expected_bytes - len(buffer))
            except TimeoutError:
                continue
            except OSError:
                return

            if not chunk:
                return

            buffer += chunk
            if len(buffer) < expected_bytes:
                continue

            with self.last_coordinates_lock:
                self.last_coordinates = struct.unpack("!ii", buffer[:expected_bytes])

            buffer = buffer[expected_bytes:]