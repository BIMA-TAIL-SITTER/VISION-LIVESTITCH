"""A separate spawned OS process owns each UAV's geometry and raster state."""
import io
import logging
from pathlib import Path
from queue import Empty, Full
import time
import cv2
import numpy as np
from PIL import Image
from .geometry import Calibration
from .models import EngineConfig, FrameMeta
from .storage import atomic_write, append_json, write_json
from .stitcher import LiveEngine

log = logging.getLogger(__name__)


def decode_preview(payload, meta):
    # Inspect compressed dimensions before OpenCV allocates a decoded raster.
    with Image.open(io.BytesIO(payload)) as header:
        if header.format != "JPEG" or header.size != (meta.preview_width, meta.preview_height):
            raise ValueError("preview JPEG dimensions/format invalid")
    image = cv2.imdecode(np.frombuffer(payload, np.uint8), cv2.IMREAD_COLOR)
    if image is None:
        raise ValueError("JPEG decode failed")
    return image


def worker_main(config_dict, calibration_dict, input_queue, output_queue, stop_event,
                output_dir, max_queue_age_s):
    logging.basicConfig(level=logging.INFO, format="%(asctime)s %(processName)s %(levelname)s %(message)s")
    cv2.setNumThreads(1)  # Avoid each UAV multiplying the host's full OpenCV thread pool.
    cfg = EngineConfig.model_validate(config_dict)
    calibration = Calibration.model_validate(calibration_dict) if calibration_dict else None
    root = Path(output_dir)
    root.mkdir(parents=True, exist_ok=True)
    counters = {"processed":0, "stitched":0, "rejected":0, "registration_failed":0, "errors":0, "stale_dropped":0}
    last_publish = 0.0

    def emit(value):
        write_json(root/"status.json", value)
        try:
            output_queue.put_nowait(value)
        except Full:
            # Durable status.json/events remain authoritative if UI consumes slowly.
            pass

    def publish(engine, final=False, reason=None):
        image = engine.canvas.render(cfg.preview_max_side)
        if image is None:
            return
        ok, jpeg = cv2.imencode(".jpg", image, [cv2.IMWRITE_JPEG_QUALITY, 88])
        if not ok:
            raise IOError("preview encode failed")
        folder = root/"segments"/f"{engine.segment:04d}"
        atomic_write(folder/"preview.jpg", jpeg.tobytes())
        atomic_write(root/"preview.jpg", jpeg.tobytes())
        description = {"segment":engine.segment, "reason_started":engine.segment_start_reason,
            "reason_closed":reason, "stream_epoch":engine.epoch, "georeferenced":False,
            "canvas_origin":list(engine.canvas.origin), "canvas_shape":engine.canvas.shape,
            "preview_size":[image.shape[1],image.shape[0]], "final":final,
            "last_frame":engine.last_frame.model_dump(mode="json") if engine.last_frame else None}
        if final:
            raster = engine.canvas.render()
            ok, png = cv2.imencode(".png", raster)
            if not ok:
                raise IOError("mosaic encode failed")
            atomic_write(folder/"mosaic.png", png.tobytes())
            # Explicit alpha/coverage artifact: dark pixels remain valid.
            ok, coverage = cv2.imencode(".png", (engine.canvas.weight>0).astype(np.uint8)*255)
            if not ok:
                raise IOError("coverage encode failed")
            atomic_write(folder/"coverage.png", coverage.tobytes())
        write_json(folder/"segment.json", description)

    engine = LiveEngine(cfg, calibration, lambda eng, reason: publish(eng, True, reason))
    emit({"state":"ready", **counters})
    fatal = None
    try:
        while True:
            try:
                item = input_queue.get(timeout=0.2)
            except Empty:
                if stop_event.is_set():
                    break
                continue
            meta_data, payload, received_ns = item
            start = time.monotonic_ns()
            queue_age_ms = (start-received_ns)/1e6
            meta = FrameMeta.model_validate(meta_data)
            counters["processed"] += 1
            try:
                if queue_age_ms > max_queue_age_s*1000:
                    counters["stale_dropped"] += 1
                    result = {"status":"stale_dropped", "reason":"queue_age_limit"}
                else:
                    result = engine.process(decode_preview(payload, meta), meta)
                    counters[result["status"]] += 1
                    if result["status"] == "stitched" and time.monotonic()-last_publish >= cfg.publish_interval_s:
                        publish(engine)
                        last_publish = time.monotonic()
            except Exception as exc:
                counters["errors"] += 1
                result = {"status":"error", "reason":f"{type(exc).__name__}: {exc}"}
                log.exception("Frame %s failed", meta.key)
            event = {"state":"running", "frame_id":meta.frame_id, "stream_epoch":str(meta.stream_epoch),
                "queue_age_ms":queue_age_ms, "worker_wall_ms":(time.monotonic_ns()-start)/1e6,
                "receive_to_processed_ms":(time.monotonic_ns()-received_ns)/1e6,
                "segment":engine.segment, "canvas_shape":engine.canvas.shape,
                "result":result, **counters}
            append_json(root/"events.jsonl", event)
            emit(event)
    except Exception as exc:
        fatal = f"{type(exc).__name__}: {exc}"
        log.exception("Worker fatal error")
    finally:
        try:
            publish(engine, True, "shutdown")
        except Exception as exc:
            fatal = f"final_publish: {exc}"
            log.exception("Final mosaic could not be saved")
        emit({"state":"failed" if fatal else "stopped", "error":fatal,
              "segment":engine.segment, "canvas_shape":engine.canvas.shape, **counters})
