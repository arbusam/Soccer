import asyncio
import websockets

HOST = "127.0.0.1"
PORT = 8765

latest_log: str | None = None  # only ever keep the newest log
new_log_event: asyncio.Event = asyncio.Event()
clients: set[websockets.WebSocketServerProtocol] = set()

# Ball position received from clients (e.g., from mouse clicks in simulate.py)
ball_position: tuple[float | None, float | None] = (None, None)

async def send_log():
    while True:
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
    global latest_log
    latest_log = log
    new_log_event.set()

async def init_server():
    asyncio.create_task(send_log())
    async with websockets.serve(handle_client, HOST, PORT):
        print(f"Log server running at ws://{HOST}:{PORT}")
        await asyncio.Future()