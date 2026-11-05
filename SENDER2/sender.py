"""Onboard capture/replay, original archive first, preview UDP second."""
import logging
from pathlib import Path
import socket
import threading
import time
import uuid
import cv2
import numpy as np
from .models import FrameMeta, Telemetry
from .protocol import pack_frame
from .storage import archive_frame, append_json
from .telemetry import MAVLinkReader

log = logging.getLogger(__name__)


class TransmissionError(OSError):
    """Original is safely archived, but preview delivery failed."""


class CameraPump(threading.Thread):
    """Continuously drains UVC. Exposes latest decoded frame with host read timestamp."""
    def __init__(self, index, width, height):
        super().__init__(name="sender2-camera",daemon=True)
        self.cap = cv2.VideoCapture(index)
        if not self.cap.isOpened():
            raise RuntimeError("camera could not be opened")
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH,width)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT,height)
        self.cap.set(cv2.CAP_PROP_BUFFERSIZE,1)
        self.lock = threading.Lock()
        self.stop_event = threading.Event()
        self.latest = None
        self.error = None
        self.sequence = 0

    def run(self):
        try:
            while not self.stop_event.is_set():
                ok, frame = self.cap.read()
                stamp_ns, utc_ns = time.monotonic_ns(), time.time_ns()
                if not ok:
                    raise IOError("camera read failed")
                with self.lock:
                    self.sequence += 1
                    self.latest = (self.sequence,frame,stamp_ns,utc_ns)
        except Exception as exc:
            with self.lock:
                self.error = str(exc)
            log.exception("Camera capture stopped")
        finally:
            self.cap.release()

    def take(self, last_sequence):
        with self.lock:
            if self.error:
                raise IOError(self.error)
            if self.latest is None or self.latest[0] == last_sequence:
                return None
            seq, frame, stamp, utc = self.latest
            return seq, frame.copy(), stamp, utc

    def stop(self):
        self.stop_event.set()
        self.join(timeout=3)
        if self.is_alive():
            log.error("Camera driver read blocked; capture thread did not stop within deadline")


class FrameSender:
    def __init__(self, host, port, mission_id, uav_id, camera_id, archive_dir,
                 calibration_id=None, preview_width=1280, quality=85,
                 max_mbps=8, simulation=False, send_datagram=None):
        self.host,self.port = host,port
        self.mission_id,self.uav_id,self.camera_id = mission_id,uav_id,camera_id
        self.archive_dir,self.calibration_id = Path(archive_dir),calibration_id
        self.preview_width,self.quality = preview_width,quality
        self.max_mbps,self.simulation = max_mbps,simulation
        self.epoch,self.started_ns = uuid.uuid4(),time.time_ns()
        self.frame_id = 0
        self.socket = socket.socket(socket.AF_INET,socket.SOCK_DGRAM) if send_datagram is None else None
        self.send_datagram = send_datagram or (lambda data:self.socket.sendto(data,(self.host,self.port)))
        if not 160 <= preview_width <= 2048 or not 1 <= quality <= 100 or max_mbps <= 0:
            raise ValueError("invalid sender limits")

    def send(self, image, telemetry=None, capture_ns=None, utc_ns=None,
             original_bytes=None, extension=".png", source_filename=None):
        mono = time.monotonic_ns() if capture_ns is None else capture_ns
        utc = time.time_ns() if utc_ns is None else utc_ns
        h,w = image.shape[:2]
        scale = min(1,self.preview_width/w)
        size = (max(1,round(w*scale)), max(1,round(h*scale)))
        meta = FrameMeta(mission_id=self.mission_id,uav_id=self.uav_id,stream_epoch=self.epoch,
            stream_started_unix_ns=self.started_ns,frame_id=self.frame_id,
            capture_timestamp_unix_ns=utc,capture_timestamp_monotonic_ns=mono,
            capture_time_source="replay_read_completed" if self.simulation else "host_read_completed",
            camera_id=self.camera_id,calibration_id=self.calibration_id,
            capture_width=w,capture_height=h,preview_width=size[0],preview_height=size[1],
            telemetry=telemetry or Telemetry(),simulation=self.simulation,source_filename=source_filename)
        # Consume identity before I/O; failed sends never reuse the same frame ID.
        self.frame_id += 1
        if original_bytes is None:
            ok, original = cv2.imencode(".png",image)
            if not ok:
                raise IOError("lossless original archive encode failed")
            original_bytes,extension = original.tobytes(),".png"
        archive_frame(self.archive_dir,meta,original_bytes,extension,
            "original_file_bytes" if self.simulation else "lossless_decoded_camera_frame")
        preview = cv2.resize(image,size,interpolation=cv2.INTER_AREA)
        ok,encoded = cv2.imencode(".jpg",preview,[cv2.IMWRITE_JPEG_QUALITY,self.quality])
        if not ok:
            raise IOError("preview encode failed; original remains archived")
        # Pacing avoids a burst of thousands of UDP fragments. Bandwidth is per UAV.
        deadline = time.monotonic()
        count = 0
        try:
            for datagram in pack_frame(meta,encoded.tobytes()):
                pause = deadline-time.monotonic()
                if pause > 0:
                    time.sleep(pause)
                self.send_datagram(datagram)
                deadline = max(deadline,time.monotonic())+len(datagram)*8/(self.max_mbps*1e6)
                count += 1
        except Exception as exc:
            append_json(self.archive_dir/"transmission_errors.jsonl",{"frame":meta.model_dump(mode="json"),"error":str(exc)})
            raise TransmissionError(str(exc)) from exc
        return meta,count

    def close(self):
        if self.socket:
            self.socket.close()


def add_arguments(parser):
    parser.add_argument("--host",required=True)
    parser.add_argument("--port",type=int,required=True)
    parser.add_argument("--mission", "--flight", dest="mission", default="flight01",
                        help="Must match ground flight/mission name")
    parser.add_argument("--uav",choices=["UAV1","UAV2"],required=True)
    parser.add_argument("--camera-id",default="camera1")
    parser.add_argument("--calibration-id")
    parser.add_argument("--archive-dir",default="SENDER2/onboard_archive")
    parser.add_argument("--camera-index",type=int,default=0)
    parser.add_argument("--width",type=int,default=1920)
    parser.add_argument("--height",type=int,default=1080)
    parser.add_argument("--preview-width",type=int,default=1280)
    parser.add_argument("--quality",type=int,default=85)
    parser.add_argument("--interval",type=float,default=0.5)
    parser.add_argument("--max-mbps",type=float,default=8)
    parser.add_argument("--fc",help="MAVLink serial port or udpin endpoint")
    parser.add_argument("--baud",type=int,default=57600)
    parser.add_argument("--replay-dir",help="Explicit simulation; existing photo bytes are archived unchanged")
    parser.add_argument("--limit",type=int,default=0,help="0 means no limit")


def run(args):
    if args.interval <= 0 or args.limit < 0 or not 1 <= args.port <= 65535:
        raise ValueError("invalid interval, port or limit")
    if not args.replay_dir and not args.fc:
        raise ValueError("camera mode requires --fc; use --replay-dir for offline testing")
    sender = FrameSender(args.host,args.port,args.mission,args.uav,args.camera_id,args.archive_dir,
        args.calibration_id,args.preview_width,args.quality,args.max_mbps,bool(args.replay_dir))
    reader = None
    camera = None
    try:
        if args.replay_dir:
            paths = sorted(p for p in Path(args.replay_dir).iterdir() if p.suffix.lower() in {".jpg",".jpeg",".png",".bmp",".tif",".tiff"})
            if args.limit:
                paths = paths[:args.limit]
            if not paths:
                raise ValueError("replay directory contains no images")
            for path in paths:
                start = time.monotonic()
                data = path.read_bytes()
                image = cv2.imdecode(np.frombuffer(data,np.uint8),cv2.IMREAD_COLOR)
                if image is None:
                    raise ValueError(f"cannot decode {path.name}")
                # Sidecars generated by SENDER2 retain measured telemetry for replay.
                # Ordinary image files have unknown telemetry, never fake pose/GPS.
                telemetry = Telemetry()
                sidecar = path.with_suffix(".json")
                if sidecar.exists():
                    import json
                    record = json.loads(sidecar.read_text(encoding="utf-8"))
                    telemetry = Telemetry.model_validate(record.get("frame",{}).get("telemetry",{}))
                try:
                    meta,count = sender.send(image,telemetry,original_bytes=data,extension=path.suffix,source_filename=path.name)
                    log.info("%s frame=%d chunks=%d original=%s",args.uav,meta.frame_id,count,path.name)
                except TransmissionError:
                    log.exception("Preview send failed; original archived, continuing replay")
                time.sleep(max(0,args.interval-(time.monotonic()-start)))
        else:
            reader = MAVLinkReader(args.fc,args.baud)
            reader.start()
            camera = CameraPump(args.camera_index,args.width,args.height)
            camera.start()
            last_sequence = 0
            while not args.limit or sender.frame_id < args.limit:
                start = time.monotonic()
                latest = camera.take(last_sequence)
                if latest is None:
                    time.sleep(0.01)
                    continue
                last_sequence,image,stamp,utc = latest
                if time.monotonic_ns()-stamp > 1_000_000_000:
                    log.warning("Camera sample stale; waiting for a fresh frame")
                    continue
                try:
                    meta,count = sender.send(image,reader.buffer.snapshot(stamp),stamp,utc)
                    log.info("%s frame=%d chunks=%d attitude_age_ms=%s",args.uav,meta.frame_id,count,meta.telemetry.attitude_age_ms)
                except TransmissionError:
                    log.exception("Preview send failed; original archived, continuing camera capture")
                time.sleep(max(0,args.interval-(time.monotonic()-start)))
    except KeyboardInterrupt:
        log.info("Sender stopping")
    finally:
        if camera:
            camera.stop()
        if reader:
            reader.stop()
        sender.close()
