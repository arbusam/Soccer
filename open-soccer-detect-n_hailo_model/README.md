# Open Soccer YOLO26n detect — Hailo-8 deploy folder

Place the compiled `model.hef` here (AI HAT+ 26 TOPS / `hw_arch=hailo8`).

## Build on an x86 host (not the Pi)

```bash
# 1) Export detect dataset + train + ONNX
python training/export_label_studio.py --format detect
python training/train_detect.py --size n --device xpu   # or --device cpu

# 2) Compile HEF with Hailo DFC installed
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
python test_model.py
```

`test_model.py` and `camera.py` load `open-soccer-detect-n_hailo_model/model.hef` via HailoRT.
