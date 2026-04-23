"""Host a fake websocket log stream for ``simulate.py --connect`` testing."""

import asyncio
import math

import send_log

PITCH_WIDTH = 2430
PITCH_HEIGHT = 1820
FPS = 30
PORT = send_log.PORT


def clamp(value, lower, upper):
    return max(lower, min(upper, value))


def build_log_line(t_seconds):
    """Return one CSV frame in the format expected by simulate.py."""
    bot_x = 650 + 320 * math.cos(t_seconds * 0.8)
    bot_y = 910 + 220 * math.sin(t_seconds * 1.1)
    yaw = math.degrees(math.atan2(math.cos(t_seconds * 1.1), -math.sin(t_seconds * 0.8))) % 360

    ball_x = 1215 + 500 * math.cos(t_seconds * 0.55)
    ball_y = 910 + 350 * math.sin(t_seconds * 0.7)

    other_bots = [
        (
            1800 + 180 * math.cos(t_seconds * 0.6),
            600 + 140 * math.sin(t_seconds * 0.9),
        ),
        (
            1700 + 210 * math.sin(t_seconds * 0.75),
            1220 + 170 * math.cos(t_seconds * 0.5),
        ),
        (
            1180 + 260 * math.cos(t_seconds * 0.45),
            420 + 110 * math.sin(t_seconds * 1.3),
        ),
    ]

    values = [
        clamp(int(bot_x), 0, PITCH_WIDTH),
        clamp(int(bot_y), 0, PITCH_HEIGHT),
        round(yaw, 2),
        clamp(int(ball_x), 0, PITCH_WIDTH),
        clamp(int(ball_y), 0, PITCH_HEIGHT),
    ]
    for other_x, other_y in other_bots:
        values.extend(
            (
                clamp(int(other_x), 0, PITCH_WIDTH),
                clamp(int(other_y), 0, PITCH_HEIGHT),
            )
        )
    return ",".join(str(value) for value in values)


async def stream_test_world():
    send_log.start_server_background()
    await asyncio.sleep(0.05)

    print(f"Test websocket server running on ws://0.0.0.0:{PORT}")
    print(f"Connect with: python simulate.py --connect 127.0.0.1:{PORT}")

    start_time = asyncio.get_running_loop().time()
    frame_period = 1.0 / FPS

    while True:
        elapsed = asyncio.get_running_loop().time() - start_time
        send_log.update_latest_log(build_log_line(elapsed))
        await asyncio.sleep(frame_period)


if __name__ == "__main__":
    try:
        asyncio.run(stream_test_world())
    except KeyboardInterrupt:
        print("\nStopped test websocket server.")
