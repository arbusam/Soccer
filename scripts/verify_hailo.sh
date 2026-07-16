#!/usr/bin/env bash
# Verify Hailo-8 / AI HAT+ is visible on this machine (run on the Pi).
set -euo pipefail

echo "=== Hailo device nodes ==="
if ls /dev/hailo* 2>/dev/null; then
  echo "OK: found Hailo device node(s)"
else
  echo "FAIL: no /dev/hailo* — is the AI HAT+ seated and hailo driver loaded?"
  exit 1
fi

echo
echo "=== hailortcli fw-control identify ==="
if ! command -v hailortcli >/dev/null 2>&1; then
  echo "FAIL: hailortcli not in PATH. Install with: sudo apt install hailo-all"
  exit 1
fi
hailortcli fw-control identify

echo
echo "=== Python hailo_platform ==="
python3 - <<'PY'
try:
    import hailo_platform
    print(f"OK: hailo_platform from {hailo_platform.__file__}")
except ImportError as exc:
    raise SystemExit(
        "FAIL: Python hailo_platform missing. "
        "Use system Python / a venv with --system-site-packages after installing hailo-all."
    ) from exc
PY

echo
echo "Hailo verification passed."
