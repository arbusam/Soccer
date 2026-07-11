import json
import secrets
import socket
import threading
import time

DEFAULT_PORT = 5005
DEFAULT_PEER_TIMEOUT_S = 0.5
DEFAULT_SOCKET_TIMEOUT_S = 0.05
BROADCAST_ADDR = "255.255.255.255"


class Peer:
    """Symmetric UDP broadcast peer for bot-to-bot state sharing."""

    def __init__(self, port: int = DEFAULT_PORT, peer_timeout_s: float = DEFAULT_PEER_TIMEOUT_S):
        self.port = port
        self.peer_timeout_s = peer_timeout_s
        self.bot_id = secrets.token_hex(8)

        self._socket: socket.socket | None = None
        self._running = False
        self._receive_thread: threading.Thread | None = None
        self._lock = threading.Lock()
        self._last_message: dict | None = None
        self._last_received_at: float | None = None

    def start(self) -> None:
        if self._running:
            return

        udp_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        udp_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        udp_socket.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
        udp_socket.settimeout(DEFAULT_SOCKET_TIMEOUT_S)
        udp_socket.bind(("0.0.0.0", self.port))
        self._socket = udp_socket

        self._running = True
        self._receive_thread = threading.Thread(target=self._receive_loop, daemon=True)
        self._receive_thread.start()

    def stop(self) -> None:
        self._running = False

        if self._receive_thread is not None:
            self._receive_thread.join(timeout=0.5)
            self._receive_thread = None

        if self._socket is not None:
            try:
                self._socket.close()
            except OSError:
                pass
            self._socket = None

        with self._lock:
            self._last_message = None
            self._last_received_at = None

    def send(self, payload: dict) -> None:
        if self._socket is None:
            return

        packet = dict(payload)
        packet["bot_id"] = self.bot_id
        try:
            self._socket.sendto(
                json.dumps(packet, separators=(",", ":")).encode("utf-8"),
                (BROADCAST_ADDR, self.port),
            )
        except OSError:
            pass

    def receive(self) -> dict | None:
        with self._lock:
            if self._last_message is None or self._last_received_at is None:
                return None
            if time.monotonic() - self._last_received_at > self.peer_timeout_s:
                return None
            return self._last_message

    def _receive_loop(self) -> None:
        while self._running:
            if self._socket is None:
                time.sleep(DEFAULT_SOCKET_TIMEOUT_S)
                continue

            try:
                packet, _addr = self._socket.recvfrom(65535)
            except TimeoutError:
                continue
            except OSError:
                if self._running:
                    time.sleep(DEFAULT_SOCKET_TIMEOUT_S)
                continue

            try:
                message = json.loads(packet.decode("utf-8"))
            except (UnicodeDecodeError, json.JSONDecodeError):
                continue

            if not isinstance(message, dict):
                continue
            if message.get("bot_id") == self.bot_id:
                continue

            with self._lock:
                self._last_message = message
                self._last_received_at = time.monotonic()
