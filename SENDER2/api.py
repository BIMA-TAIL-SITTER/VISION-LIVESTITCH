"""Read-only base-station API; all stitching runs in spawned processes."""
from contextlib import asynccontextmanager
import json
import re
from fastapi import FastAPI, HTTPException
from fastapi.responses import FileResponse, JSONResponse


def create_app(runtime, manage_lifecycle=True):
    @asynccontextmanager
    async def lifespan(app):
        if manage_lifecycle:
            runtime.start()
        try:
            yield
        finally:
            if manage_lifecycle:
                runtime.stop()

    app = FastAPI(title="SENDER2 independent UAV live mosaics", version="2.0.0", lifespan=lifespan)

    def directory(uav_id):
        if uav_id not in runtime.streams:
            raise HTTPException(404, "Unknown UAV")
        return runtime.streams[uav_id]["directory"]

    @app.get("/health")
    def health():
        status = runtime.snapshot()
        healthy = bool(status["uavs"]) and not status["closing"] and all(
            s["worker_alive"] and s["worker"].get("state") in {"ready","running"}
            for s in status["uavs"].values())
        return JSONResponse({"healthy":healthy, **status}, status_code=200 if healthy else 503)

    @app.get("/uavs")
    def uavs():
        return runtime.snapshot()

    @app.get("/uavs/{uav_id}/status")
    def status(uav_id: str):
        directory(uav_id)
        return runtime.snapshot()["uavs"][uav_id]

    @app.get("/uavs/{uav_id}/preview.jpg")
    def preview(uav_id: str):
        path = directory(uav_id)/"preview.jpg"
        if not path.exists():
            raise HTTPException(404, "No accepted mosaic yet; inspect stream status")
        return FileResponse(path, media_type="image/jpeg", headers={"Cache-Control":"no-store"})

    @app.get("/uavs/{uav_id}/segments")
    def segments(uav_id: str):
        return [json.loads(p.read_text(encoding="utf-8"))
                for p in sorted((directory(uav_id)/"segments").glob("*/segment.json"))]

    @app.get("/uavs/{uav_id}/segments/{segment_id}/{asset}")
    def segment_asset(uav_id: str, segment_id: str, asset: str):
        if not re.fullmatch(r"[0-9]{4,8}",segment_id) or asset not in {"preview.jpg","mosaic.png","coverage.png","segment.json"}:
            raise HTTPException(404,"Unknown asset")
        path = directory(uav_id)/"segments"/segment_id/asset
        if not path.is_file():
            raise HTTPException(404,"Asset unavailable; full mosaic is finalized on segment close/shutdown")
        return FileResponse(path, headers={"Cache-Control":"no-store"})

    return app
