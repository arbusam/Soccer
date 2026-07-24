#!/usr/bin/env python3
"""Label Studio ML backend that pre-annotates with a trained Ultralytics YOLO detect model.

Serves the Label Studio ML HTTP API on port 9090 (override with --port / LS_ML_PORT):
  GET  /health
  POST /setup
  POST /predict

Images are resolved from LABEL_STUDIO_DATA_DIR (default: ../label-studio-data) so the
backend can run on the same host as Label Studio without an API token.

Example:
  .venv/bin/python training/ls_yolo_backend.py \\
      --weights training/runs/open-soccer-detect-n/weights/best.pt
"""

from __future__ import annotations

import argparse
import json
import logging
import os
import re
import sys
import xml.etree.ElementTree as ET
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from typing import Any
from urllib.parse import urlparse

REPO_ROOT = Path(__file__).resolve().parents[1]
DEFAULT_DATA_DIR = REPO_ROOT / "label-studio-data"
DEFAULT_WEIGHTS = (
    REPO_ROOT / "training" / "runs" / "open-soccer-detect-m" / "weights" / "best.pt"
)
DEFAULT_PORT = 9090
DEFAULT_CONF = 0.25
MODEL_VERSION = "open-soccer-detect-m"

logger = logging.getLogger("ls_yolo_backend")

_STATE: dict[str, Any] = {
    "model": None,
    "weights": None,
    "conf": DEFAULT_CONF,
    "data_dir": DEFAULT_DATA_DIR,
    "from_name": "label",
    "to_name": "image",
    "allowed_labels": {"Ball", "Bot"},
}


def _resolve_image_path(image_uri: str, data_dir: Path) -> Path:
    """Map Label Studio `/data/...` URIs (or plain paths) to local files."""
    uri = (image_uri or "").strip()
    if not uri:
        raise FileNotFoundError("empty image URI")

    parsed = urlparse(uri)
    if parsed.scheme in ("http", "https"):
        # Same-host Label Studio media URLs still end in /data/...
        path = parsed.path
    else:
        path = uri

    if path.startswith("/data/"):
        local = data_dir / "media" / path[len("/data/") :]
    else:
        candidate = Path(path)
        local = candidate if candidate.is_file() else data_dir / "media" / path.lstrip("/")

    if not local.is_file():
        raise FileNotFoundError(f"image not found: {uri} -> {local}")
    return local


def _parse_label_config(label_config: str | None) -> tuple[str, str, set[str]]:
    """Extract RectangleLabels from_name/to_name and allowed Label values."""
    from_name = "label"
    to_name = "image"
    labels: set[str] = set()
    if not label_config:
        return from_name, to_name, {"Ball", "Bot"}

    try:
        root = ET.fromstring(label_config)
    except ET.ParseError:
        logger.warning("could not parse label_config XML; using defaults")
        return from_name, to_name, {"Ball", "Bot"}

    for rect in root.iter("RectangleLabels"):
        from_name = rect.attrib.get("name", from_name)
        to_name = rect.attrib.get("toName", to_name)
        for label in rect.iter("Label"):
            value = label.attrib.get("value")
            if value:
                labels.add(value)
        break

    if not labels:
        labels = {"Ball", "Bot"}
    return from_name, to_name, labels


_THRESHOLD_RE = re.compile(
    r'model_score_threshold\s*=\s*["\']([0-9.]+)["\']', re.IGNORECASE
)


def _conf_from_config(label_config: str | None, default: float) -> float:
    if not label_config:
        return default
    match = _THRESHOLD_RE.search(label_config)
    if not match:
        return default
    try:
        return float(match.group(1))
    except ValueError:
        return default


def _load_model(weights: Path):
    from ultralytics import YOLO

    logger.info("loading YOLO weights from %s", weights)
    return YOLO(str(weights))


def _predict_task(task: dict[str, Any]) -> dict[str, Any]:
    model = _STATE["model"]
    data_dir: Path = _STATE["data_dir"]
    conf: float = _STATE["conf"]
    from_name: str = _STATE["from_name"]
    to_name: str = _STATE["to_name"]
    allowed: set[str] = _STATE["allowed_labels"]

    image_uri = task.get("data", {}).get("image")
    if not image_uri:
        return {"result": [], "score": 0.0, "model_version": MODEL_VERSION}

    path = _resolve_image_path(image_uri, data_dir)
    results = model.predict(str(path), conf=conf, verbose=False)
    if not results:
        return {"result": [], "score": 0.0, "model_version": MODEL_VERSION}

    result0 = results[0]
    boxes = result0.boxes
    names = result0.names or model.names or {}
    orig_h, orig_w = result0.orig_shape[:2] if result0.orig_shape is not None else (0, 0)

    regions: list[dict[str, Any]] = []
    scores: list[float] = []
    if boxes is None:
        return {"result": regions, "score": 0.0, "model_version": MODEL_VERSION}

    for i in range(len(boxes)):
        score = float(boxes.conf[i])
        if score < conf:
            continue
        cls_id = int(boxes.cls[i])
        model_label = names.get(cls_id, str(cls_id))
        if model_label not in allowed:
            continue

        x, y, w, h = boxes.xywhn[i].tolist()
        regions.append(
            {
                "from_name": from_name,
                "to_name": to_name,
                "type": "rectanglelabels",
                "original_width": int(orig_w),
                "original_height": int(orig_h),
                "image_rotation": 0,
                "value": {
                    "rotation": 0,
                    "x": (x - w / 2.0) * 100.0,
                    "y": (y - h / 2.0) * 100.0,
                    "width": w * 100.0,
                    "height": h * 100.0,
                    "rectanglelabels": [model_label],
                },
                "score": score,
            }
        )
        scores.append(score)

    avg = sum(scores) / len(scores) if scores else 0.0
    return {"result": regions, "score": avg, "model_version": MODEL_VERSION}


class Handler(BaseHTTPRequestHandler):
    protocol_version = "HTTP/1.1"

    def log_message(self, fmt: str, *args: Any) -> None:
        logger.info("%s - %s", self.address_string(), fmt % args)

    def _read_json(self) -> dict[str, Any]:
        length = int(self.headers.get("Content-Length", "0") or 0)
        if length <= 0:
            return {}
        raw = self.rfile.read(length)
        if not raw:
            return {}
        return json.loads(raw.decode("utf-8"))

    def _send_json(self, payload: dict[str, Any], status: int = 200) -> None:
        body = json.dumps(payload).encode("utf-8")
        self.send_response(status)
        self.send_header("Content-Type", "application/json")
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def do_GET(self) -> None:  # noqa: N802
        path = urlparse(self.path).path
        if path in ("/", "/health"):
            self._send_json(
                {
                    "status": "UP",
                    "model_class": "OpenSoccerYolo",
                    "model_version": MODEL_VERSION,
                    "weights": str(_STATE["weights"]),
                }
            )
            return
        self._send_json({"error": "not found"}, status=404)

    def do_POST(self) -> None:  # noqa: N802
        path = urlparse(self.path).path
        try:
            data = self._read_json()
        except json.JSONDecodeError as exc:
            self._send_json({"error": f"invalid JSON: {exc}"}, status=400)
            return

        if path == "/setup":
            schema = data.get("schema") or data.get("label_config")
            from_name, to_name, labels = _parse_label_config(schema)
            _STATE["from_name"] = from_name
            _STATE["to_name"] = to_name
            _STATE["allowed_labels"] = labels
            _STATE["conf"] = _conf_from_config(schema, _STATE["conf"])
            logger.info(
                "setup: from_name=%s to_name=%s labels=%s conf=%.3f",
                from_name,
                to_name,
                sorted(labels),
                _STATE["conf"],
            )
            self._send_json({"model_version": MODEL_VERSION})
            return

        if path == "/predict":
            tasks = data.get("tasks") or []
            label_config = data.get("label_config")
            if label_config:
                from_name, to_name, labels = _parse_label_config(label_config)
                _STATE["from_name"] = from_name
                _STATE["to_name"] = to_name
                _STATE["allowed_labels"] = labels
                _STATE["conf"] = _conf_from_config(label_config, _STATE["conf"])

            results = []
            for task in tasks:
                try:
                    results.append(_predict_task(task))
                except FileNotFoundError as exc:
                    logger.error("%s", exc)
                    results.append(
                        {"result": [], "score": 0.0, "model_version": MODEL_VERSION}
                    )
                except Exception:
                    logger.exception("predict failed for task %s", task.get("id"))
                    results.append(
                        {"result": [], "score": 0.0, "model_version": MODEL_VERSION}
                    )
            self._send_json({"results": results})
            return

        if path == "/webhook":
            # Training hooks are unused; acknowledge so Label Studio is happy.
            self._send_json({"status": "ok", "result": {}}, status=201)
            return

        self._send_json({"error": "not found"}, status=404)


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--weights",
        type=Path,
        default=Path(os.environ.get("LS_ML_WEIGHTS", DEFAULT_WEIGHTS)),
        help="Ultralytics .pt checkpoint (Ball/Bot detect)",
    )
    parser.add_argument(
        "--data-dir",
        type=Path,
        default=Path(os.environ.get("LABEL_STUDIO_DATA_DIR", DEFAULT_DATA_DIR)),
        help="Label Studio data dir (for /data/... media resolution)",
    )
    parser.add_argument(
        "--host",
        default=os.environ.get("LS_ML_HOST", "0.0.0.0"),
        help="Bind address",
    )
    parser.add_argument(
        "--port",
        type=int,
        default=int(os.environ.get("LS_ML_PORT", DEFAULT_PORT)),
        help="Listen port (Label Studio default ML URL uses 9090)",
    )
    parser.add_argument(
        "--conf",
        type=float,
        default=float(os.environ.get("LS_ML_CONF", DEFAULT_CONF)),
        help="Default confidence threshold (overridden by model_score_threshold in config)",
    )
    parser.add_argument(
        "-v",
        "--verbose",
        action="store_true",
        help="Debug logging",
    )
    args = parser.parse_args(argv)

    logging.basicConfig(
        level=logging.DEBUG if args.verbose else logging.INFO,
        format="%(asctime)s %(levelname)s %(name)s: %(message)s",
    )

    weights = args.weights.resolve()
    if not weights.is_file():
        logger.error("weights not found: %s", weights)
        return 1

    data_dir = args.data_dir.resolve()
    if not data_dir.is_dir():
        logger.error("Label Studio data dir not found: %s", data_dir)
        return 1

    _STATE["weights"] = weights
    _STATE["data_dir"] = data_dir
    _STATE["conf"] = args.conf
    _STATE["model"] = _load_model(weights)

    server = ThreadingHTTPServer((args.host, args.port), Handler)
    logger.info(
        "Label Studio YOLO ML backend listening on http://%s:%s (weights=%s)",
        args.host,
        args.port,
        weights,
    )
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        logger.info("shutting down")
        server.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
