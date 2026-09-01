#!/usr/bin/env python3
"""Find val images that also appear (or nearly appear) in the YOLO train split.

The Label Studio export shuffles frames independently, so consecutive video
frames can land in both splits. This script reports:

- exact file duplicates (SHA-256)
- near-duplicates via difference hash (dHash)

Example:
  python training/check_split_overlap.py
  python training/check_split_overlap.py --dataset training/datasets/open-soccer-detect
  python training/check_split_overlap.py --max-distance 20 --copy-matches /tmp/overlap
"""

from __future__ import annotations

import argparse
import hashlib
import shutil
from pathlib import Path

import cv2
import numpy as np

REPO_ROOT = Path(__file__).resolve().parents[1]
DEFAULT_DATASET = REPO_ROOT / "training" / "datasets" / "open-soccer-detect"
IMAGE_SUFFIXES = {".jpg", ".jpeg", ".png", ".bmp", ".webp", ".tif", ".tiff"}


def _list_images(folder: Path) -> list[Path]:
    if not folder.is_dir():
        raise FileNotFoundError(f"Image folder not found: {folder}")
    images = [
        path
        for path in sorted(folder.iterdir())
        if path.is_file() and path.suffix.lower() in IMAGE_SUFFIXES
    ]
    if not images:
        raise RuntimeError(f"No images in {folder}")
    return images


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for chunk in iter(lambda: handle.read(1 << 20), b""):
            digest.update(chunk)
    return digest.hexdigest()


def _dhash_bits(path: Path, hash_size: int) -> np.ndarray:
    """Return a packed 1-D uint8 bit vector of length hash_size * hash_size."""
    image = cv2.imread(str(path), cv2.IMREAD_GRAYSCALE)
    if image is None:
        raise RuntimeError(f"Failed to read image: {path}")
    resized = cv2.resize(image, (hash_size + 1, hash_size), interpolation=cv2.INTER_AREA)
    diff = resized[:, 1:] > resized[:, :-1]
    return np.packbits(diff.reshape(-1))


def _copy_pair(dest_dir: Path, val_path: Path, train_path: Path, distance: int, index: int) -> None:
    pair_dir = dest_dir / f"{index:04d}_d{distance}_{val_path.stem}"
    pair_dir.mkdir(parents=True, exist_ok=True)
    shutil.copy2(val_path, pair_dir / f"val{val_path.suffix.lower()}")
    shutil.copy2(train_path, pair_dir / f"train{train_path.suffix.lower()}")
    (pair_dir / "match.txt").write_text(
        f"distance={distance}\nval={val_path}\ntrain={train_path}\n",
        encoding="utf-8",
    )


def check_overlap(
    dataset: Path,
    hash_size: int,
    max_distance: int,
    top_k: int,
    copy_matches: Path | None,
) -> int:
    train_dir = dataset / "images" / "train"
    val_dir = dataset / "images" / "val"
    train_images = _list_images(train_dir)
    val_images = _list_images(val_dir)

    print(f"Dataset: {dataset}")
    print(f"Train images: {len(train_images)}")
    print(f"Val images:   {len(val_images)}")
    print(f"dHash size:   {hash_size}x{hash_size} ({hash_size * hash_size} bits)")
    print(f"Max Hamming:  {max_distance}")
    print()

    train_hashes = {path: _sha256(path) for path in train_images}
    hash_to_train: dict[str, list[Path]] = {}
    for path, digest in train_hashes.items():
        hash_to_train.setdefault(digest, []).append(path)

    exact_pairs: list[tuple[Path, Path]] = []
    for val_path in val_images:
        digest = _sha256(val_path)
        for train_path in hash_to_train.get(digest, []):
            exact_pairs.append((val_path, train_path))

    print(f"Exact duplicates: {len(exact_pairs)}")
    if exact_pairs:
        for val_path, train_path in exact_pairs:
            print(f"  {val_path.name}  ==  {train_path.name}")
        print()

    print("Computing dHashes...")
    train_dhashes = np.unpackbits(
        np.stack([_dhash_bits(path, hash_size) for path in train_images]),
        axis=1,
    )
    val_dhashes = np.unpackbits(
        np.stack([_dhash_bits(path, hash_size) for path in val_images]),
        axis=1,
    )

    near_hits: list[tuple[int, Path, Path]] = []
    leaked_val: set[Path] = set()
    for val_path, val_hash in zip(val_images, val_dhashes, strict=True):
        distances = np.bitwise_xor(train_dhashes, val_hash).sum(axis=1).astype(np.int32)
        order = np.argsort(distances)
        for rank in order[:top_k]:
            distance = int(distances[rank])
            if distance > max_distance:
                break
            train_path = train_images[int(rank)]
            near_hits.append((distance, val_path, train_path))
            leaked_val.add(val_path)

    near_hits.sort(key=lambda item: (item[0], item[1].name, item[2].name))
    print(f"Near-duplicate pairs (Hamming ≤ {max_distance}): {len(near_hits)}")
    print(f"Val images with a train match: {len(leaked_val)} / {len(val_images)}")
    if near_hits:
        print()
        print(f"{'dist':>4}  {'val':<40}  train")
        for distance, val_path, train_path in near_hits:
            print(f"{distance:4d}  {val_path.name:<40}  {train_path.name}")

    if copy_matches is not None and near_hits:
        copy_matches.mkdir(parents=True, exist_ok=True)
        for index, (distance, val_path, train_path) in enumerate(near_hits, start=1):
            _copy_pair(copy_matches, val_path, train_path, distance, index)
        print(f"\nCopied {len(near_hits)} pair folders → {copy_matches}")

    leaked_exact = {val_path for val_path, _ in exact_pairs}
    return len(leaked_val | leaked_exact)


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Check whether val images (or near-duplicates) also exist in train."
    )
    parser.add_argument(
        "--dataset",
        type=Path,
        default=DEFAULT_DATASET,
        help="YOLO dataset root with images/train and images/val",
    )
    parser.add_argument(
        "--hash-size",
        type=int,
        default=16,
        help="dHash grid size (bits = size²). Larger is more precise.",
    )
    parser.add_argument(
        "--max-distance",
        type=int,
        default=16,
        help=(
            "Max Hamming distance to count as a near-duplicate. "
            "16/256 bits (~6%%) catches same-scene video frames; "
            "lower it if you only want almost-identical stills."
        ),
    )
    parser.add_argument(
        "--top-k",
        type=int,
        default=3,
        help="Max train matches reported per val image (after the distance filter)",
    )
    parser.add_argument(
        "--copy-matches",
        type=Path,
        default=None,
        help="If set, copy each matching val/train pair into this directory for visual review",
    )
    args = parser.parse_args()

    if args.hash_size < 4:
        raise SystemExit("--hash-size must be >= 4")
    if args.max_distance < 0:
        raise SystemExit("--max-distance must be >= 0")
    if args.top_k < 1:
        raise SystemExit("--top-k must be >= 1")

    leaked = check_overlap(
        dataset=args.dataset.resolve(),
        hash_size=args.hash_size,
        max_distance=args.max_distance,
        top_k=args.top_k,
        copy_matches=args.copy_matches.resolve() if args.copy_matches else None,
    )
    if leaked:
        raise SystemExit(1)


if __name__ == "__main__":
    main()
