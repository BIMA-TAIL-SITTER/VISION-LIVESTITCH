"""Atomic artifacts and append-only manifests. All paths are generated internally."""
import hashlib
import json
import os
import time
from pathlib import Path
import uuid


def atomic_write(path: Path, data: bytes):
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    tmp = path.with_name(f".{path.name}.{uuid.uuid4().hex}.part")
    try:
        with tmp.open("xb") as stream:
            stream.write(data)
            stream.flush()
            os.fsync(stream.fileno())
        # Windows readers/antivirus may briefly hold the destination without
        # FILE_SHARE_DELETE. Keep the old complete file visible while retrying;
        # persistent permission failures still propagate to the caller.
        for attempt in range(8):
            try:
                os.replace(tmp, path)
                break
            except PermissionError:
                if attempt == 7:
                    raise
                time.sleep(min(0.01 * 2**attempt, 0.2))
    finally:
        tmp.unlink(missing_ok=True)


def write_json(path, value):
    atomic_write(Path(path), json.dumps(value, indent=2, allow_nan=False).encode("utf-8"))


def append_json(path, value):
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("a", encoding="utf-8") as stream:
        stream.write(json.dumps(value, allow_nan=False) + "\n")
        stream.flush()


def archive_frame(root, meta, data: bytes, extension: str, kind: str, *, include_mission=True):
    if extension.lower() not in {".jpg", ".jpeg", ".png", ".bmp", ".tif", ".tiff"}:
        raise ValueError("unsupported archive format")
    base = Path(root) / meta.mission_id if include_mission else Path(root)
    folder = base / meta.uav_id / str(meta.stream_epoch)
    name = f"{meta.frame_id:012d}"
    path = folder / f"{name}{extension.lower()}"
    atomic_write(path, data)
    record = {"frame": meta.model_dump(mode="json"), "kind": kind,
              "sha256": hashlib.sha256(data).hexdigest(), "file": path.name}
    write_json(folder / f"{name}.json", record)
    append_json(folder / "manifest.jsonl", record)
    return path
