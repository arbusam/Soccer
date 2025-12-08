# log_server.py
import asyncio
import websockets

HOST = "127.0.0.1"
PORT = 8765

latest_log: str | None = None  # only ever keep the newest log
new_log_event: asyncio.Event = asyncio.Event()
clients: set[websockets.WebSocketServerProtocol] = set()

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
    clients.add(ws)
    try:
        await ws.wait_closed()
    finally:
        clients.discard(ws)

def update_latest_log(log: str) -> None:
    global latest_log
    latest_log = log
    new_log_event.set()

async def init_server():
    asyncio.create_task(send_log())
    async with websockets.serve(handle_client, HOST, PORT):
        print(f"Log server running at ws://{HOST}:{PORT}")
        await asyncio.Future()