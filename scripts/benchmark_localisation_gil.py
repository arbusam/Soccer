"""Artificial mutex-contention benchmark; run as python -m scripts.benchmark_localisation_gil PATH."""

import argparse
import json
import time

from lib import lidar, timing


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("output")
    args = parser.parse_args()
    recorder = timing.Recorder(
        warmup=0.05,
        duration=3,
        metadata={"workload": "five artificial 200 ms mutex holds"},
    )
    recorder.attach_native(lidar)
    timing.active = recorder
    try:
        lidar.test_mcl_start(2430, 1820)
        time.sleep(0.06)
        for _ in range(5):
            lidar.test_mcl_hold_mutex(200)
            timing.call("binding.pose", lidar.get_pose)
            time.sleep(0.02)
    finally:
        lidar.test_mcl_stop()
        summary = recorder.finish(args.output, lidar)
        timing.active = None
    print(
        json.dumps(
            {
                "runtime_gil_released": lidar.runtime_gil_released,
                "heartbeat": summary["metrics"]["heartbeat.delay"],
                "pose_call": summary["metrics"]["binding.pose"],
                "dropped": summary["dropped"],
            },
            indent=2,
        )
    )


if __name__ == "__main__":
    main()
