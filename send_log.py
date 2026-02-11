import asyncio
import threading
import websockets

HOST = "127.0.0.1"
PORT = 8765

latest_log: str | None = None  # only ever keep the newest log
clients: set[websockets.WebSocketServerProtocol] = set()
new_log_event: asyncio.Event | None = None
_server_loop: asyncio.AbstractEventLoop | None = None

# Ball position received from clients (e.g., from mouse clicks in simulate.py)
ball_position: tuple[float | None, float | None] = (None, None)

async def send_log():
    while True:
        # Created inside the running server loop in init_server()
        assert new_log_event is not None
        await new_log_event.wait()
        new_log_event.clear()
        msg = latest_log
        if msg is None:
            continue
        if clients:
            await asyncio.gather(
                *(c.send(msg) for c in list(clients)),
                return_exceptions=True,
            )

async def handle_client(ws, path=None):
    global ball_position
    clients.add(ws)
    try:
        async for message in ws:
            # Expect messages in format "ball:x,y"
            if message.startswith("ball:"):
                try:
                    coords = message[5:].split(",")
                    x = float(coords[0])
                    y = float(coords[1])
                    ball_position = (x, y)
                except (ValueError, IndexError):
                    pass
    except websockets.ConnectionClosed:
        pass
    finally:
        clients.discard(ws)


def get_ball_position() -> tuple[float | None, float | None]:
    """Return the current ball position as set by clients."""
    return ball_position

def update_latest_log(log: str) -> None:
    """
    Thread-safe: may be called from non-async code / different threads.
    """
    global latest_log

    def _apply_update() -> None:
        global latest_log
        latest_log = log
        if new_log_event is not None:
            new_log_event.set()

    # If the server loop is running (even in another thread), schedule the update there.
    if _server_loop is not None and _server_loop.is_running():
        _server_loop.call_soon_threadsafe(_apply_update)
        return

    # Fallback (e.g., server not started yet).
    _apply_update()

async def init_server():
    global new_log_event, _server_loop
    _server_loop = asyncio.get_running_loop()
    new_log_event = asyncio.Event()
    asyncio.create_task(send_log())
    async with websockets.serve(handle_client, HOST, PORT):
        print(f"Log server running at ws://{HOST}:{PORT}")
        await asyncio.Future()

def start_server_background(daemon: bool = True) -> threading.Thread:
    """
    Start the websocket log server in a background thread with its own event loop.
    """
    t = threading.Thread(target=lambda: asyncio.run(init_server()), daemon=daemon)
    t.start()
    return t