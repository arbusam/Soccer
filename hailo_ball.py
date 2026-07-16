"""Hailo-8 (AI HAT+) YOLO26 detect inference for ball bearing/distance."""

from __future__ import annotations

import math
from pathlib import Path

import numpy as np
import yaml

BALL_CLASS_NAME = "ball"
DEFAULT_HAIL_MODEL_DIR = Path(__file__).resolve().parent / "open-soccer-detect-n_hailo_model"
DEFAULT_CLASS_NAMES = {
    0: "Ball",
    1: "Blue Goal",
    2: "Yellow Goal",
    3: "Bot",
}


def letterbox(
    image: np.ndarray,
    imgsz: int = 640,
    pad_value: int = 114,
) -> tuple[np.ndarray, float, tuple[int, int]]:
    """Resize with aspect ratio preserved and gray pad to imgsz×imgsz (NHWC RGB)."""
    height, width = image.shape[:2]
    scale = min(imgsz / height, imgsz / width)
    new_w = int(round(width * scale))
    new_h = int(round(height * scale))
    # Local import keeps module importable without cv2 when unused.
    import cv2

    resized = cv2.resize(image, (new_w, new_h), interpolation=cv2.INTER_LINEAR)
    canvas = np.full((imgsz, imgsz, 3), pad_value, dtype=np.uint8)
    top = (imgsz - new_h) // 2
    left = (imgsz - new_w) // 2
    canvas[top : top + new_h, left : left + new_w] = resized
    return canvas, scale, (left, top)


def _load_metadata(model_dir: Path) -> dict:
    meta_path = model_dir / "metadata.yaml"
    if not meta_path.is_file():
        return {"names": DEFAULT_CLASS_NAMES, "imgsz": 640, "task": "detect"}
    with meta_path.open(encoding="utf-8") as handle:
        meta = yaml.safe_load(handle) or {}
    names = meta.get("names") or DEFAULT_CLASS_NAMES
    if isinstance(names, list):
        names = {i: name for i, name in enumerate(names)}
    elif isinstance(names, dict):
        names = {int(k): v for k, v in names.items()}
    meta["names"] = names
    return meta


class HailoBallDetector:
    """Run a compiled YOLO26 detect HEF on HailoRT and return ball candidates."""

    def __init__(
        self,
        model_dir: str | Path = DEFAULT_HAIL_MODEL_DIR,
        conf: float = 0.25,
    ):
        self.model_dir = Path(model_dir)
        self.hef_path = self.model_dir / "model.hef"
        if not self.hef_path.is_file():
            raise FileNotFoundError(
                f"Hailo HEF not found: {self.hef_path}\n"
                "Train/export ONNX with training/train_detect.py, then compile with "
                "training/compile_hailo.py on an x86 host with the Hailo DFC, and copy "
                f"{self.model_dir.name}/ to this machine."
            )

        try:
            from hailo_platform import (
                HEF,
                VDevice,
                ConfigureParams,
                InferVStreams,
                InputVStreamParams,
                OutputVStreamParams,
                FormatType,
                HailoStreamInterface,
            )
        except ImportError as exc:
            raise ImportError(
                "hailo_platform is not importable. On the Pi install hailo-all and use a "
                "venv created with --system-site-packages."
            ) from exc

        self.conf = float(conf)
        self.meta = _load_metadata(self.model_dir)
        self.names = self.meta.get("names") or DEFAULT_CLASS_NAMES
        imgsz = self.meta.get("imgsz") or 640
        if isinstance(imgsz, (list, tuple)):
            self.imgsz = int(imgsz[0])
        else:
            self.imgsz = int(imgsz)

        self._HEF = HEF
        self._VDevice = VDevice
        self._ConfigureParams = ConfigureParams
        self._InferVStreams = InferVStreams
        self._InputVStreamParams = InputVStreamParams
        self._OutputVStreamParams = OutputVStreamParams
        self._FormatType = FormatType
        self._HailoStreamInterface = HailoStreamInterface

        self._hef = HEF(str(self.hef_path))
        self._target = VDevice()
        configure_params = ConfigureParams.create_from_hef(
            hef=self._hef,
            interface=HailoStreamInterface.PCIe,
        )
        self._network_group = self._target.configure(self._hef, configure_params)[0]
        self._network_group_params = self._network_group.create_params()

        self._input_vstream_info = self._hef.get_input_vstream_infos()[0]
        self._output_vstream_infos = self._hef.get_output_vstream_infos()
        self._input_name = self._input_vstream_info.name

        self._input_vstreams_params = InputVStreamParams.make_from_network_group(
            self._network_group,
            quantized=False,
            format_type=FormatType.UINT8,
        )
        self._output_vstreams_params = OutputVStreamParams.make_from_network_group(
            self._network_group,
            quantized=False,
            format_type=FormatType.FLOAT32,
        )

        self._infer = InferVStreams(
            self._network_group,
            self._input_vstreams_params,
            self._output_vstreams_params,
        )
        self._activated = self._network_group.activate(self._network_group_params)
        self._activated.__enter__()
        self._infer.__enter__()
        self._closed = False

    @property
    def names_map(self):
        return self.names

    def close(self) -> None:
        if self._closed:
            return
        self._closed = True
        try:
            self._infer.__exit__(None, None, None)
        except Exception:
            pass
        try:
            self._activated.__exit__(None, None, None)
        except Exception:
            pass
        target = getattr(self, "_target", None)
        if target is None:
            return
        if hasattr(target, "release"):
            try:
                target.release()
            except Exception:
                pass
        elif hasattr(target, "__exit__"):
            try:
                target.__exit__(None, None, None)
            except Exception:
                pass

    def __enter__(self):
        return self

    def __exit__(self, exc_type, exc, tb):
        self.close()
        return False

    def _is_ball_class(self, class_id: int) -> bool:
        class_id = int(class_id)
        class_name = self.names.get(class_id)
        if class_name is not None:
            return str(class_name).strip().lower() == BALL_CLASS_NAME
        return class_id == 0

    def predict(self, frame_rgb: np.ndarray) -> list[dict]:
        """Return detections as dicts with xyxy (original frame), conf, cls."""
        if self._closed:
            raise RuntimeError("HailoBallDetector is closed")

        padded, scale, (pad_left, pad_top) = letterbox(frame_rgb, self.imgsz)
        input_data = {self._input_name: np.expand_dims(padded, axis=0)}
        raw = self._infer.infer(input_data)

        # Prefer the first output tensor; YOLO26 end-to-end is (1, 300, 6).
        output = None
        for info in self._output_vstream_infos:
            tensor = np.asarray(raw[info.name])
            if tensor.ndim >= 2:
                output = tensor
                break
        if output is None:
            return []

        output = np.asarray(output)
        if output.ndim == 3:
            output = output[0]
        if output.ndim != 2 or output.shape[-1] < 6:
            raise RuntimeError(
                f"Unexpected Hailo output shape {output.shape}; expected (N, 6) "
                "YOLO26 detections [x1,y1,x2,y2,conf,cls] in letterbox space."
            )

        detections: list[dict] = []
        frame_h, frame_w = frame_rgb.shape[:2]
        for row in output:
            conf = float(row[4])
            if conf < self.conf:
                continue
            class_id = int(row[5])
            x1 = (float(row[0]) - pad_left) / scale
            y1 = (float(row[1]) - pad_top) / scale
            x2 = (float(row[2]) - pad_left) / scale
            y2 = (float(row[3]) - pad_top) / scale
            x1 = min(max(x1, 0.0), frame_w - 1.0)
            y1 = min(max(y1, 0.0), frame_h - 1.0)
            x2 = min(max(x2, 0.0), frame_w - 1.0)
            y2 = min(max(y2, 0.0), frame_h - 1.0)
            if x2 <= x1 or y2 <= y1:
                continue
            detections.append(
                {
                    "xyxy": (x1, y1, x2, y2),
                    "confidence": conf,
                    "class_id": class_id,
                }
            )
        return detections

    def best_ball(self, frame_rgb: np.ndarray) -> dict | None:
        """Highest-confidence ball detection with centre / radial_pixels."""
        frame_h, frame_w = frame_rgb.shape[:2]
        candidates = []
        for det in self.predict(frame_rgb):
            if not self._is_ball_class(det["class_id"]):
                continue
            x1, y1, x2, y2 = det["xyxy"]
            centre_x = (x1 + x2) / 2.0
            centre_y = (y1 + y2) / 2.0
            candidates.append(
                {
                    "bbox": (
                        int(round(x1)),
                        int(round(y1)),
                        int(round(x2 - x1)),
                        int(round(y2 - y1)),
                    ),
                    "centre": (centre_x, centre_y),
                    "radial_pixels": math.hypot(
                        centre_x - (frame_w / 2.0),
                        centre_y - (frame_h / 2.0),
                    ),
                    "confidence": det["confidence"],
                    "polygon": None,
                }
            )
        if not candidates:
            return None
        return max(candidates, key=lambda item: item["confidence"])
