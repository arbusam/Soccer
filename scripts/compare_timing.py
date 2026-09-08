"""Compare two timing summaries: python scripts/compare_timing.py A.summary.json B.summary.json."""

import argparse
import json
from pathlib import Path


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("before", type=Path)
    parser.add_argument("after", type=Path)
    args = parser.parse_args()
    before = json.loads(args.before.read_text())
    after = json.loads(args.after.read_text())
    print(f"Dropped samples: before={before['dropped']} after={after['dropped']}")
    print(
        f"Capture seconds: before={before['captured_seconds']:.2f} after={after['captured_seconds']:.2f}"
    )
    print(
        f"{'Metric':42} {'A p99 ms':>10} {'B p99 ms':>10} {'A max ms':>10} {'B max ms':>10}"
    )
    for name in sorted(before["metrics"].keys() | after["metrics"].keys()):
        a, b = before["metrics"].get(name), after["metrics"].get(name)
        if a is None or b is None:
            print(
                f"{name:42} samples: A={a['count'] if a else 0}, B={b['count'] if b else 0}"
            )
            continue
        print(
            f"{name:42} {a['p99_ms']:10.3f} {b['p99_ms']:10.3f} {a['max_ms']:10.3f} {b['max_ms']:10.3f}"
        )
        for threshold in (25, 40):
            key = f"over_{threshold}ms"
            if key in a and key in b:
                print(
                    f"  >{threshold} ms: {a[key]['count']} ({a[key]['percent']:.2f}%) -> "
                    f"{b[key]['count']} ({b[key]['percent']:.2f}%)"
                )
    for label, report in [("A", before), ("B", after)]:
        print(f"{label} event rates Hz: {report['event_rates_hz']}")
        print(
            f"{label} confident pose %: {report.get('confident_pose_percent', 'unavailable')}; "
            f"scan gate paused %: {report.get('scan_gate_paused_percent', 'unavailable')}"
        )


if __name__ == "__main__":
    main()
