# Open Soccer YOLO26n detect — Hailo-8 deploy folder

Place the compiled `model.hef` here (AI HAT+ 26 TOPS / `hw_arch=hailo8`).

## Build on an x86 host (not the Pi)

```bash
# 1) Optional: rewrite rotated LS boxes to AABBs (stop Label Studio first)
python training/export_label_studio.py --update-label-studio --apply

# 2) Export detect dataset + train + ONNX
python training/export_label_studio.py
python training/train.py --size n   # project .venv; default device=xpu

# 3) Compile HEF with Hailo DFC installed
# Cuts at Mul_2 + Sigmoid (separate box/score outputs). Do NOT use
# /model.23/Transpose — concatenating boxes+scores breaks INT8 scores.
python training/compile_hailo.py \
  --onnx training/exports/open-soccer-detect-n/model.onnx \
  --calib-dir training/datasets/open-soccer-detect/images/train \
  --out-dir open-soccer-detect-n_hailo_model \
  --hw-arch hailo8
```

## On the Pi

```bash
bash scripts/verify_hailo.sh
# copy this folder (model.hef + metadata.yaml) onto the Pi, then:
python tests/model.py
```

`tests/model.py` and `lib/camera.py` load `open-soccer-detect-n_hailo_model/model.hef` via HailoRT.
