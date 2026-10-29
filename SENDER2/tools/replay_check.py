"""Bounded local replay through real UDP + two spawned workers; writes a run report."""
import argparse
from concurrent.futures import ThreadPoolExecutor
import json
import logging
from pathlib import Path
import socket
import time
import cv2
import numpy as np
from SENDER2.models import GroundConfig, EngineConfig, UAVConfig
from SENDER2.runtime import GroundRuntime
from SENDER2.sender import FrameSender
from SENDER2.storage import write_json


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--dataset",required=True)
    parser.add_argument("--count",type=int,default=12,help="images per UAV")
    parser.add_argument("--output",default="SENDER2/test_artifacts/replay")
    parser.add_argument("--interval",type=float,default=0.8)
    args = parser.parse_args()
    paths = sorted(p for p in Path(args.dataset).iterdir() if p.suffix.lower() in {".jpg",".jpeg",".png"})[:args.count]
    if not paths or args.count < 1 or args.interval < 0:
        raise ValueError("invalid replay input")
    reservations = [socket.socket(socket.AF_INET,socket.SOCK_DGRAM) for _ in range(2)]
    for s in reservations: s.bind(("127.0.0.1",0))
    ports = [s.getsockname()[1] for s in reservations]
    for s in reservations: s.close()
    cfg = GroundConfig(mission_id="audit_replay",bind_host="127.0.0.1",output_dir=args.output,
        allow_simulation=True,engine=EngineConfig(require_attitude=False,require_calibration=False),
        uavs=[UAVConfig(uav_id="UAV1",camera_id="camera1",port=ports[0]),
              UAVConfig(uav_id="UAV2",camera_id="camera2",port=ports[1])])
    runtime = GroundRuntime(cfg)
    runtime.start()
    def feed(index):
        sender = FrameSender("127.0.0.1",ports[index],cfg.mission_id,f"UAV{index+1}",f"camera{index+1}",
            runtime.root/"onboard_archive",preview_width=1280,max_mbps=25,simulation=True)
        try:
            # Both sources deliberately use the same dataset to test isolation,
            # not as a claim of real dual-aircraft flight validation.
            for path in paths:
                t = time.monotonic()
                data = path.read_bytes()
                image = cv2.imdecode(np.frombuffer(data,np.uint8),cv2.IMREAD_COLOR)
                sender.send(image,original_bytes=data,extension=path.suffix,source_filename=path.name)
                time.sleep(max(0,args.interval-(time.monotonic()-t)))
        finally:
            sender.close()
    try:
        with ThreadPoolExecutor(max_workers=2) as pool:
            list(pool.map(feed,range(2)))
        # Allow incomplete datagrams to expire, then drain all enqueued frames.
        deadline = time.monotonic()+45
        while time.monotonic()<deadline:
            status = runtime.snapshot()
            if all(s["worker"].get("processed",0)>=s["network"].get("enqueued",0)
                   and s["pending_bytes"]==0 for s in status["uavs"].values()):
                break
            time.sleep(0.1)
    finally:
        runtime.stop()
    report = {"dataset":str(Path(args.dataset).resolve()),"files":[p.name for p in paths],
              "frames_sent_per_uav":len(paths),"configuration":cfg.model_dump(mode="json"),
              "shutdown":json.loads((runtime.root/"shutdown.json").read_text()),"metrics":{}}
    for uav in cfg.uavs:
        path = runtime.root/uav.uav_id/"events.jsonl"
        events = [json.loads(line) for line in path.read_text().splitlines()] if path.exists() else []
        wall = [event["worker_wall_ms"] for event in events]
        report["metrics"][uav.uav_id] = {"events":len(events),
            "worker_wall_p50_ms":float(np.median(wall)) if wall else None,
            "worker_wall_p95_ms":float(np.percentile(wall,95)) if wall else None}
    write_json(runtime.root/"replay_report.json",report)
    print(json.dumps({"report":str(runtime.root/"replay_report.json"),"metrics":report["metrics"],
                      "uavs":report["shutdown"]["uavs"]},indent=2))
    if any(s["worker"].get("stitched",0)==0 or s["worker"].get("errors",0)>0
           or s["worker"].get("state")!="stopped" for s in report["shutdown"]["uavs"].values()):
        raise SystemExit("Replay failed; inspect report")


if __name__=="__main__":
    logging.basicConfig(level=logging.INFO)
    main()
