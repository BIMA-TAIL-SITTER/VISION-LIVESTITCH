"""Bounded network admission, process isolation and deterministic shutdown."""
from collections import Counter
from datetime import datetime, timezone
import json
import logging
import multiprocessing as mp
from pathlib import Path
from queue import Empty, Full
import socket
import threading
import time
import uuid
from .geometry import Calibration
from .protocol import Reassembler
from .storage import archive_frame, append_json, write_json
from .worker import worker_main

log = logging.getLogger(__name__)


class StreamOrder:
    """Within an epoch IDs increase; a restart must use a newer start timestamp.

Clock rollback fails visibly instead of silently mixing sessions. A new ground
run is required if the sender clock is intentionally set backwards.
"""
    def __init__(self):
        self.epoch = None
        self.started = 0
        self.frame_id = -1

    def admit(self, meta):
        epoch = str(meta.stream_epoch)
        if epoch != self.epoch:
            if meta.stream_started_unix_ns <= self.started:
                return False
            self.epoch, self.started, self.frame_id = epoch, meta.stream_started_unix_ns, -1
        elif meta.stream_started_unix_ns != self.started:
            return False
        if meta.frame_id <= self.frame_id:
            return False
        self.frame_id = meta.frame_id
        return True


class GroundRuntime:
    def __init__(self, config):
        self.cfg = config
        run_id = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ")+"_"+uuid.uuid4().hex[:8]
        self.root = Path(config.output_dir).resolve()/config.mission_id/run_id
        self.ctx = mp.get_context("spawn")
        self.stop_network = threading.Event()
        self.streams = {}
        self.lock = threading.RLock()
        self.started = False
        self.closing = False

    def start(self):
        if self.started:
            raise RuntimeError("runtime already started")
        self.root.mkdir(parents=True, exist_ok=False)
        write_json(self.root/"config.json", self.cfg.model_dump(mode="json"))
        try:
            for uav in self.cfg.uavs:
                cal = Calibration.load(uav.calibration_file) if uav.calibration_file else None
                if self.cfg.engine.require_calibration and cal is None:
                    raise ValueError(f"{uav.uav_id}: calibration_file required; use explicit replay config for uncalibrated testing")
                if cal and cal.camera_id != uav.camera_id:
                    raise ValueError(f"{uav.uav_id}: configured camera_id differs from calibration")
                sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
                try:
                    sock.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 4*1024*1024)
                    sock.bind((self.cfg.bind_host, uav.port))
                    sock.settimeout(0.2)
                except Exception:
                    sock.close()
                    raise
                queue = self.ctx.Queue(maxsize=self.cfg.queue_size)
                results = self.ctx.Queue(maxsize=64)
                event = self.ctx.Event()
                directory = self.root/uav.uav_id
                process = self.ctx.Process(name=f"SENDER2-{uav.uav_id}", target=worker_main,
                    args=(self.cfg.engine.model_dump(), cal.model_dump() if cal else None, queue, results,
                          event, str(directory), self.cfg.max_queue_age_s))
                stream = {"config":uav, "socket":sock, "queue":queue, "results":results, "stop":event,
                    "process":process, "thread":None, "directory":directory, "network":Counter(),
                    "last_error":None, "order":StreamOrder(), "status":{"state":"starting"},
                    "assembler":Reassembler(self.cfg.mission_id,uav.uav_id,self.cfg.reassembly_timeout_s)}
                self.streams[uav.uav_id] = stream
                process.start()
                thread = threading.Thread(target=self._receive, args=(uav.uav_id,), name=f"udp-{uav.uav_id}", daemon=True)
                stream["thread"] = thread
                thread.start()
            self.started = True
        except Exception:
            self.stop()
            raise

    def _receive(self, uav_id):
        stream = self.streams[uav_id]
        assembler = stream["assembler"]
        while not self.stop_network.is_set():
            try:
                packet, address = stream["socket"].recvfrom(1500)
            except socket.timeout:
                with self.lock:
                    assembler.cleanup()
                continue
            except OSError:
                if not self.stop_network.is_set():
                    log.exception("UDP receiver failed for %s", uav_id)
                    with self.lock:
                        stream["last_error"] = "UDP socket failure"
                break
            try:
                with self.lock:
                    stream["network"]["datagrams"] += 1
                    completed = assembler.feed(packet,address)
                if completed is None:
                    continue
                meta, payload = completed
                received_ns = time.monotonic_ns()
                if meta.camera_id != stream["config"].camera_id:
                    raise ValueError("unexpected camera identity")
                if meta.simulation and not self.cfg.allow_simulation:
                    raise ValueError("simulation frame rejected by flight configuration")
                if not stream["order"].admit(meta):
                    with self.lock:
                        stream["network"]["late_or_duplicate"] += 1
                    continue
                # Archive every completed admitted preview even when live queue is full.
                archive_frame(self.root/"received_previews", meta, payload, ".jpg", "received_compressed_preview")
                if not stream["process"].is_alive():
                    raise RuntimeError("worker unavailable; preview archived, restart ground run")
                try:
                    # Drop newest on overload: preserve the admitted ordered chain.
                    # Age limits plus cache recovery handle gaps; never block the network on a full queue.
                    stream["queue"].put_nowait((meta.model_dump(mode="json"), payload, received_ns))
                    with self.lock:
                        stream["network"]["enqueued"] += 1
                except Full:
                    with self.lock:
                        stream["network"]["queue_dropped"] += 1
                    append_json(stream["directory"]/"network_events.jsonl",
                                {"event":"queue_full", "frame":meta.model_dump(mode="json")})
            except Exception as exc:
                log.exception("Receive/admission error for %s", uav_id)
                with self.lock:
                    stream["network"]["errors"] += 1
                    stream["last_error"] = str(exc)

    def snapshot(self):
        with self.lock:
            output = {}
            for name, s in self.streams.items():
                while True:
                    try:
                        s["status"] = s["results"].get_nowait()
                    except (Empty, ValueError, OSError):
                        break
                # Atomic durable status avoids stale UI if the result queue filled.
                status_file = s["directory"]/"status.json"
                if status_file.exists():
                    s["status"] = json.loads(status_file.read_text(encoding="utf-8"))
                output[name] = {"worker_alive":s["process"].is_alive(),
                    "worker_exitcode":s["process"].exitcode, "port":s["config"].port,
                    "network":dict(s["network"]), "reassembly":dict(s["assembler"].stats),
                    "pending_bytes":s["assembler"].bytes_pending, "last_error":s["last_error"],
                    "worker":s["status"]}
            return {"mission_id":self.cfg.mission_id, "run_dir":str(self.root),
                    "closing":self.closing, "uavs":output}

    def stop(self, timeout_s=20):
        if self.closing:
            return
        self.closing = True
        self.stop_network.set()
        for s in self.streams.values():
            s["socket"].close()
        for s in self.streams.values():
            if s["thread"]:
                s["thread"].join(timeout=3)
                if s["thread"].is_alive():
                    s["last_error"] = "receiver_shutdown_timeout"
                    log.error("Receiver did not stop within deadline; archive I/O may be blocked")
            s["stop"].set()
        deadline = time.monotonic()+timeout_s
        while any(s["process"].is_alive() for s in self.streams.values()) and time.monotonic() < deadline:
            # Drain process status pipes so worker exit cannot stall on a full feeder.
            self.snapshot()
            for s in self.streams.values():
                if s["process"].pid:
                    s["process"].join(timeout=0.05)
        for name,s in self.streams.items():
            if s["process"].is_alive():
                log.error("Forced shutdown for %s; inspect last atomic snapshot", name)
                s["process"].terminate()
                s["process"].join(timeout=3)
                write_json(s["directory"]/"shutdown_error.json", {"error":"worker_timeout", "final_mosaic_guaranteed":False})
        write_json(self.root/"shutdown.json", self.snapshot())
        for s in self.streams.values():
            for q in (s["queue"],s["results"]):
                q.cancel_join_thread()
                q.close()
