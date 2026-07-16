---
name: raspberry-pi-bus-error
description: Diagnose and fix Bus error (SIGBUS) when importing numpy/cv2/torch on Raspberry Pi. Use when encountering Bus errors during Python imports, especially with numpy, opencv-python, torch, ultralytics, or picamera2 on Raspberry Pi systems.
---

# Raspberry Pi: "Bus error" when importing `numpy`/`cv2`/`torch`

## Symptom

Running Python scripts (e.g., `defence.py`, `camera.py`, or `test_model.py`) crashes with `Bus error` (SIGBUS), often before any Python traceback. With Ultralytics this can also show up as a misleading `ImportError: cannot import name 'YOLO'` / missing `__version__`, while `import torch` (or `import numpy`) actually Bus-errors.

## Cause

The Raspberry Pi kernel is configured with **16KB pages** (`getconf PAGE_SIZE` → `16384`). Some `pip` wheels for native libraries (notably `numpy`, `torch`, and packages that pull them in such as `opencv-python` / `ultralytics`) can **SIGBUS on 16KB-page systems**.

## Fix A — OS packages (camera / numpy / opencv, no pip torch)

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

## Fix B — Need `torch` / `ultralytics` (YOLO on Pi)

Pip `torch` almost always SIGBUS on 16KB pages. Switch the Pi to **4KB pages**, then reinstall:

1. Confirm: `getconf PAGE_SIZE` → `16384`
2. Add to `/boot/firmware/config.txt` (Pi OS Bookworm):
   ```
   kernel=kernel8.img
   ```
3. Reboot, confirm: `getconf PAGE_SIZE` → `4096`
4. Recreate the venv (still `--system-site-packages` for picamera2/libcamera), then:
   ```bash
   pip install torch torchvision --index-url https://download.pytorch.org/whl/cpu
   pip install ultralytics
   ```
   Do **not** pip-install `numpy` / `opencv-python` / `picamera2` if apt versions are already available via system site-packages.