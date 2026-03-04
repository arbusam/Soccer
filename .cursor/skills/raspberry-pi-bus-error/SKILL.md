---
name: raspberry-pi-bus-error
description: Diagnose and fix Bus error (SIGBUS) when importing numpy/cv2 on Raspberry Pi. Use when encountering Bus errors during Python imports, especially with numpy, opencv-python, or picamera2 on Raspberry Pi systems.
---

# Raspberry Pi: "Bus error" when importing `numpy`/`cv2`

## Symptom

Running Python scripts (e.g., `defence.py` or `camera.py`) crashes with `Bus error` (SIGBUS), often before any Python traceback.

## Cause

The Raspberry Pi kernel is configured with **16KB pages** (`getconf PAGE_SIZE` → `16384`). Some `pip` wheels for native libraries (notably `numpy`, and therefore `opencv-python`) can **SIGBUS on 16KB-page systems**. In this repo, importing `numpy` inside the project `.venv` triggered the SIGBUS during `numpy._core.multiarray` import.

## Fix

Prefer OS-packaged builds for native libraries on the Pi:

1. **Install OS packages:**
   ```bash
   sudo apt-get install python3-numpy python3-opencv python3-libcamera python3-picamera2
   ```

2. **Use system Python or system-site-packages venv:**
   - Run scripts with system Python: `python3 ...`
   - Or create a venv that uses system packages:
     ```bash
     python3 -m venv .venv --system-site-packages
     ```
   - Avoid `pip` installing `numpy`, `opencv-python`, or `picamera2` into the venv.
