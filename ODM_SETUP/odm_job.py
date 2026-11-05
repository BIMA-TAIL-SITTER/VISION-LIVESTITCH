"""Submit one UAV's original photos to local NodeODM; resume via saved job JSON."""
import argparse
import hashlib
import json
import re
from datetime import datetime, timezone
from pathlib import Path

from pyodm import Node
from pyodm.types import TaskStatus

ROOT = Path(__file__).resolve().parent


def photos_in(folder):
    photos, duplicates, seen = [], [], set()
    for path in sorted(folder.iterdir()):
        if not path.is_file() or path.suffix.lower() not in {".jpg", ".jpeg", ".tif", ".tiff", ".png"}:
            continue
        with path.open("rb") as stream:
            digest = hashlib.file_digest(stream, "sha256").hexdigest()
        if digest in seen:
            duplicates.append(path.name)
        else:
            seen.add(digest)
            photos.append(path)
    return photos, duplicates


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    commands = parser.add_subparsers(dest="command", required=True)
    commands.add_parser("check", help="Check local NodeODM API")
    submit = commands.add_parser("submit", help="Create a NEW job (one UAV only)")
    submit.add_argument("--images", type=Path, required=True)
    submit.add_argument("--name", required=True, help="Unique mission/UAV name, e.g. mission01-UAV1")
    submit.add_argument("--resolution", type=float, default=5, help="Orthophoto cm/pixel; not positional accuracy")
    for command in ("status", "download"):
        sub = commands.add_parser(command)
        sub.add_argument("--job", type=Path, required=True)
        if command == "download":
            sub.add_argument("--wait", action="store_true", help="Wait for existing task; does not resubmit")
    args = parser.parse_args()
    node = Node("127.0.0.1", 3000)
    if args.command == "check":
        info = node.info()
        print(json.dumps(vars(info), default=str, indent=2))
        return
    if args.command == "submit":
        if not re.fullmatch(r"[A-Za-z0-9][A-Za-z0-9_-]{0,79}", args.name):
            parser.error("Use 1-80 letters, digits, underscores or hyphens for --name")
        if not (args.resolution > 0 and args.resolution < 10000):
            parser.error("--resolution must be positive and finite, below 10000 cm/pixel")
        photos, duplicates = photos_in(args.images.resolve(strict=True))
        if len(photos) < 3:
            raise ValueError("Need at least 3 unique photos (mapping normally needs many more with overlap)")
        job_path = ROOT / "jobs" / (args.name + ".json")
        job_path.parent.mkdir(parents=True, exist_ok=True)
        # Reserve the name before uploading: never silently duplicate a saved task.
        with job_path.open("x", encoding="utf-8") as stream:
            record = {"name": args.name, "state": "preparing", "created_utc": datetime.now(timezone.utc).isoformat()}
            json.dump(record, stream, indent=2)
        options = {"orthophoto-resolution": args.resolution, "skip-3dmodel": True, "max-concurrency": 4}
        record.update(images=[str(p) for p in photos], duplicates_skipped=duplicates, options=options, state="uploading")
        job_path.write_text(json.dumps(record, indent=2), encoding="utf-8")
        print(f"Uploading {len(photos)} unique photos; skipped {len(duplicates)} identical copies.", flush=True)
        task = node.create_task([str(p) for p in photos], options, name=args.name, parallel_uploads=2)
        record.update(uuid=task.uuid, state="submitted")
        job_path.write_text(json.dumps(record, indent=2), encoding="utf-8")
        print(f"Job saved: {job_path}\nUUID: {task.uuid}")
        return
    record = json.loads(args.job.read_text(encoding="utf-8"))
    if "uuid" not in record:
        raise ValueError("Upload did not record UUID. Inspect http://localhost:3000 before retrying to avoid duplicates.")
    task = node.get_task(record["uuid"])
    if args.command == "status":
        print(json.dumps(vars(task.info()), default=str, indent=2))
        print("\n".join(task.output(-20)))
        return
    if args.wait:
        task.wait_for_completion(status_callback=lambda info: print(f"{info.status}: {info.progress:.1f}%", flush=True), interval=10)
    if task.info().status != TaskStatus.COMPLETED:
        raise RuntimeError("Task is not complete. Use status or download --wait.")
    # Use UUID rather than untrusted name from the JSON as a directory component.
    uuid = record["uuid"]
    if not re.fullmatch(r"[A-Za-z0-9-]+", uuid):
        raise ValueError("Invalid task UUID")
    destination = ROOT / "results" / uuid
    task.download_assets(str(destination), parallel_downloads=2)
    print(f"Results: {destination}")


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("Stopped waiting/uploading. Existing NodeODM tasks continue; inspect UI before submitting again.")
        raise SystemExit(130)
    except Exception as exc:
        print(f"ERROR: {exc}")
        raise SystemExit(1)
