import asyncio
from pathlib import Path

import send_log


async def main():
    server_thread = send_log.start_server_background()
    await asyncio.sleep(0.05)

    lines = Path("test_game_log.txt").read_text().splitlines()
    try:
        for line in lines:
            send_log.update_latest_log(line)
            await asyncio.sleep(0.1)
    finally:
        # Server runs in a background thread/event-loop and is daemonized by default.
        # Nothing to await/cancel here.
        _ = server_thread


if __name__ == "__main__":
    asyncio.run(main())