"""Replay a complete dataset through independent UAV UDP/worker pipelines.

No base-station UI or real aircraft required. Source files are never modified.
"""
import argparse
from collections import Counter
from concurrent.futures import ThreadPoolExecutor
import hashlib
import json
import logging
from pathlib import Path
import socket
import threading
import time

import cv2
import numpy as np
from SENDER2.models import EngineConfig, GroundConfig, UAVConfig
from SENDER2.runtime import GroundRuntime
from SENDER2.sender import FrameSender
from SENDER2.storage import write_json, atomic_write


def discover(dataset, keep_duplicates=False):
    files = sorted((p for p in Path(dataset).iterdir()
                    if p.is_file() and p.suffix.lower() in {".jpg", ".jpeg", ".png"}),
                   key=lambda p: (len(p.name), p.name))
    seen, selected, duplicates = {}, [], []
    for path in files:
        digest = hashlib.sha256(path.read_bytes()).hexdigest()
        if digest in seen and not keep_duplicates:
            duplicates.append({"skipped": path.name, "same_as": seen[digest]})
        else:
            selected.append(path)
            seen[digest] = path.name
    return sorted(selected), duplicates, len(files)


def statistics(values):
    return {"p50": float(np.median(values)), "p95": float(np.percentile(values, 95)),
            "max": float(max(values))} if values else None


def make_report(runtime, paths, audit, sent, elapsed, errors):
    shutdown = json.loads((runtime.root / "shutdown.json").read_text())
    report = {"dataset": audit, "files": [p.name for p in paths],
              "files_per_uav": audit.get("per_uav", {}), "configuration": runtime.cfg.model_dump(mode="json"),
              "elapsed_seconds": elapsed, "sender_errors": errors,
              "shutdown": shutdown, "uavs": {}}
    for name, state in shutdown["uavs"].items():
        event_path = runtime.result_root / name / "events.jsonl"
        events = [json.loads(line) for line in event_path.read_text().splitlines()] if event_path.exists() else []
        segments = []
        for p in sorted((runtime.result_root / name / "segments").glob("*/segment.json")):
            info = json.loads(p.read_text())
            mask = cv2.imread(str(p.parent / "coverage.png"), cv2.IMREAD_GRAYSCALE)
            info["coverage_fraction"] = float(np.count_nonzero(mask) / mask.size) if mask is not None else None
            info["accepted_frames"] = sum(e["result"]["status"] == "stitched" and e["segment"] == info["segment"] for e in events)
            info["folder"] = str(p.parent.relative_to(runtime.root))
            segments.append(info)
        report["uavs"][name] = {
            "sent": sent[name], "network": state["network"], "reassembly": state["reassembly"],
            "worker": state["worker"], "exitcode": state["worker_exitcode"],
            "worker_ms": statistics([e["worker_wall_ms"] for e in events]),
            "queue_age_ms": statistics([e["queue_age_ms"] for e in events]),
            "receive_to_processed_ms": statistics([e["receive_to_processed_ms"] for e in events]),
            "result_counts": dict(Counter(e["result"]["status"] for e in events)),
            "rejection_reasons": dict(Counter(e["result"].get("reason") for e in events if e["result"]["status"] != "stitched")),
            "segments": segments}
    write_json(runtime.root / "simulation_report.json", report)
    lines = ["# Hasil simulasi SENDER2", "", f"Dataset: `{audit['path']}`", "",
             f"Input per UAV: {json.dumps(audit.get('per_uav', {}), ensure_ascii=False)}",
             f"Durasi run termasuk shutdown: {elapsed:.1f} detik.", "",
             "Dataset per UAV tercatat di simulation_report.json; state/output masing-masing terpisah. UDP localhost; bukan uji internet atau kamera DJI. Tanpa kalibrasi/attitude, timestamp replay bukan exposure asli. Interval tetap, tidak mengikuti timestamp filename.", "",
             "| UAV | Dikirim | Diterima lengkap | Diproses | Masuk mosaik* | Gagal registrasi | Queue drop | Stale drop | Error worker | Segmen |",
             "|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|"]
    for name, data in report["uavs"].items():
        w, n = data["worker"], data["network"]
        lines.append(f"| {name} | {data['sent']} | {data['reassembly'].get('completed',0)} | {w.get('processed',0)} | {w.get('stitched',0)} | {w.get('registration_failed',0)} | {n.get('queue_dropped',0)} | {w.get('stale_dropped',0)} | {w.get('errors',0)} | {len(data['segments'])} |")
    lines += ["", "*Termasuk frame anchor pertama setiap segmen. Segmen terpisah tidak otomatis tersambung. Coverage adalah fraksi pixel berisi data dalam bounding canvas, bukan luas tanah atau akurasi geografis.", ""]
    for name, data in report["uavs"].items():
        lines += [f"## {name}", "", f"Worker ms: `{data['worker_ms']}`; queue age ms: `{data['queue_age_ms']}`.",
                  f"Status: {data['worker'].get('state')}; exit code: {data['exitcode']}; alasan penolakan: `{data['rejection_reasons']}`.", ""]
        for seg in data["segments"]:
            folder = seg["folder"].replace("\\", "/")
            lines += [f"- Segmen {seg['segment']:04d}: {seg['accepted_frames']} frame; ukuran {seg['canvas_shape']}; alasan mulai `{seg['reason_started']}`. [Preview]({folder}/preview.jpg), [mosaik]({folder}/mosaic.png), [mask]({folder}/coverage.png)."]
    lines += ["", "Waktu worker bukan latency capture-to-display. Keberhasilan runtime tidak membuktikan mosaik bebas drift/ghosting. Periksa preview dan registrasi pada data terkalibrasi sebelum digunakan untuk penerbangan."]
    atomic_write(runtime.root / "SUMMARY.md", "\n".join(lines).encode("utf-8"))
    return report


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--dataset", default="dataset/DATASET 4")
    parser.add_argument("--dataset2", help="Dataset UAV2; defaults to the same dataset as UAV1")
    parser.add_argument("--flight", help="Named flight folder with Image and Hasil, e.g. FLIGHT-01")
    parser.add_argument("--output", default="SENDER2/runs/simulation")
    parser.add_argument("--interval", type=float, default=0.8)
    parser.add_argument("--max-mbps", type=float, default=8)
    parser.add_argument("--limit", type=int, default=0, help="0 = all unique photos")
    parser.add_argument("--keep-duplicates", action="store_true")
    args = parser.parse_args()
    if args.interval <= 0 or args.max_mbps <= 0 or args.limit < 0:
        parser.error("interval/bandwidth must be positive; limit must be nonnegative")
    paths, duplicates, discovered = discover(args.dataset, args.keep_duplicates)
    if args.limit:
        paths = paths[:args.limit]
    if not paths:
        parser.error("dataset contains no supported images")
    paths2, duplicates2, discovered2 = discover(args.dataset2 or args.dataset, args.keep_duplicates)
    if args.limit:
        paths2 = paths2[:args.limit]
    if not paths2:
        parser.error("UAV2 dataset contains no supported images")
    datasets = {"UAV1": paths, "UAV2": paths2}
    audit = {"path": str(Path(args.dataset).resolve()), "discovered": discovered,
             "duplicates": duplicates, "interval_s": args.interval, "max_mbps_per_uav": args.max_mbps}
    audit["per_uav"] = {
        "UAV1": {"path": str(Path(args.dataset).resolve()), "selected": len(paths), "discovered": discovered, "duplicates": duplicates},
        "UAV2": {"path": str(Path(args.dataset2 or args.dataset).resolve()), "selected": len(paths2), "discovered": discovered2, "duplicates": duplicates2}}
    sockets = [socket.socket(socket.AF_INET, socket.SOCK_DGRAM) for _ in range(2)]
    try:
        for sock in sockets:
            sock.bind(("127.0.0.1", 0))
        ports = [sock.getsockname()[1] for sock in sockets]
    finally:
        for sock in sockets:
            sock.close()
    cfg = GroundConfig(mission_id=args.flight or "dataset_sim", flight_layout=bool(args.flight), bind_host="127.0.0.1", output_dir=args.output,
        allow_simulation=True, engine=EngineConfig(require_attitude=False, require_calibration=False),
        uavs=[UAVConfig(uav_id=f"UAV{i+1}", camera_id=f"camera{i+1}", port=p) for i, p in enumerate(ports)])
    runtime = GroundRuntime(cfg)
    sent = {"UAV1": 0, "UAV2": 0}
    cancelled = threading.Event()
    errors = []
    started = time.monotonic()
    runtime.start()
    print(f"RUN_DIR={runtime.root}\nUnique frames: UAV1={len(paths)}, UAV2={len(paths2)}", flush=True)

    def feed(index):
        name = f"UAV{index+1}"
        sender = FrameSender("127.0.0.1", ports[index], cfg.mission_id, name, f"camera{index+1}",
                             runtime.root / "onboard_archive", simulation=True, max_mbps=args.max_mbps)
        try:
            for path in datasets[name]:
                if cancelled.is_set():
                    break
                tick = time.monotonic()
                original = path.read_bytes()
                image = cv2.imdecode(np.frombuffer(original, np.uint8), cv2.IMREAD_COLOR)
                if image is None:
                    raise ValueError(f"cannot decode {path.name}")
                sender.send(image, original_bytes=original, extension=path.suffix, source_filename=path.name)
                sent[name] += 1
                cancelled.wait(max(0, args.interval - (time.monotonic() - tick)))
        finally:
            sender.close()

    try:
        with ThreadPoolExecutor(max_workers=2) as pool:
            futures = [pool.submit(feed, i) for i in range(2)]
            try:
                while not all(f.done() for f in futures):
                    status = runtime.snapshot()
                    print(json.dumps({"sent": sent.copy(), "processed": {n: s['worker'].get('processed',0) for n,s in status['uavs'].items()}}), flush=True)
                    if any(not s["worker_alive"] for s in status["uavs"].values()):
                        cancelled.set()
                        errors.append("worker exited during replay")
                    time.sleep(5)
                for future in futures:
                    try:
                        future.result()
                    except Exception as exc:
                        errors.append(f"{type(exc).__name__}: {exc}")
            finally:
                cancelled.set()
        # Wait for receiver traffic to settle, then GroundRuntime.stop drains worker queues.
        time.sleep(cfg.reassembly_timeout_s + 0.5)
    finally:
        runtime.stop()
    report = make_report(runtime, paths, audit, sent, time.monotonic()-started, errors)
    write_json(Path(args.output) / "latest_run.json", {"run_dir": str(runtime.root)})
    print(f"REPORT={runtime.root / 'simulation_report.json'}\nSUMMARY={runtime.root / 'SUMMARY.md'}", flush=True)
    if errors or any(d["reassembly"].get("completed", 0) == 0 or d["worker"].get("errors",0) or d["worker"].get("state") != "stopped" for d in report["uavs"].values()):
        raise SystemExit("Simulation encountered errors; see report")


if __name__ == "__main__":
    logging.basicConfig(level=logging.INFO)
    main()
