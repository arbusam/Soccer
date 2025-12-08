import asyncio
import send_log


async def main():
    server_task = asyncio.create_task(send_log.init_server())
    await asyncio.sleep(0.05)

    try:
        with open("test_game_log.txt", "r") as f:
            for line in f:
                send_log.update_latest_log(line)
                await asyncio.sleep(0.1)
    finally:
        server_task.cancel()
        try:
            await server_task
        except asyncio.CancelledError:
            pass


if __name__ == "__main__":
    asyncio.run(main())