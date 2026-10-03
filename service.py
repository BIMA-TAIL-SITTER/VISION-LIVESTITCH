import os
import sys
import time
import threading
from pathlib import Path
from typing import Dict, List
import asyncio
from contextlib import asynccontextmanager

from fastapi import FastAPI, UploadFile, File, WebSocket, BackgroundTasks
from fastapi.middleware.cors import CORSMiddleware
from fastapi.responses import FileResponse
from pydantic import BaseModel
from watchdog.observers import Observer
from watchdog.events import FileSystemEventHandler

# Import your existing stitcher modules
sys.path.insert(0, os.path.dirname(__file__))
from src import utilities as util
from src import Combiner
from src.flight_metadata import (
    extract_flight_metadata,
    build_data_matrix,
    GPSDistanceFilter,
    AttitudeThresholdFilter,
)
import cv2
import numpy as np
import config

# Disable OpenCL for stability
cv2.ocl.setUseOpenCL(False)

# Session configuration
class StitchConfig(BaseModel):
    sessionId: str
    auto_stitch_threshold: int = 5
    auto_stitch_enabled: bool = False
    folder_monitoring_enabled: bool = False
    output_name: str = "finalResult.png"

class StitchingSession:
    def __init__(self, session_id: str, config: StitchConfig):
        self.session_id = session_id
        self.config = config
        
        self.image_folder = Path(f"./sessions/{session_id}/images")
        self.image_folder.mkdir(parents=True, exist_ok=True)
        
        self.output_folder = Path(f"./sessions/{session_id}/output")
        self.output_folder.mkdir(parents=True, exist_ok=True)
        
        # Count existing images
        self.image_count = self._count_images()
        self.is_stitching = False
        self.should_stop = False
        self.ws_clients = []
        self.last_stitch_count = 0
        self.observer = None

        # per session filters -- for multi uav configurations
        self.gps_filter = GPSDistanceFilter(threshold_m=2.0)
        self.attitude_filter = AttitudeThresholdFilter()
        self.accepted_images: List[Path] = []
        self.accepted_metadata: List[Dict] = []
        self._state_lock = threading.Lock()
    
    def _count_images(self):
        """Count all image files in the images folder"""
        count = 0
        for ext in ['*.jpg', '*.JPG', '*.jpeg', '*.JPEG', '*.png', '*.PNG', '*.tif', '*.TIF']:
            count += len(list(self.image_folder.glob(ext)))
        return count

    def claim_stitch(self) -> bool:
        """Atomically claim the stitching slot. Returns False if a stitch is already running."""
        with self._state_lock:
            if self.is_stitching:
                return False
            self.is_stitching = True
            self.last_stitch_count = len(self.accepted_images)
            return True

    def finish_stitch(self):
        with self._state_lock:
            self.is_stitching = False

# Watchdog handler
class SessionFolderHandler(FileSystemEventHandler):
    def __init__(self, session_id: str, debounce_s=1.0, stable_wait=0.5, stable_tries=6):
        self.session_id = session_id
        self.debounce_s = debounce_s
        self.stable_wait = stable_wait
        self.stable_tries = stable_tries
        self.last_called = {}
        
    def _should_call(self, path):
        """Check if enough time has passed since last call (debouncing)"""
        now = time.time()
        last = self.last_called.get(path, 0)
        if now - last < self.debounce_s:
            return False
        self.last_called[path] = now
        return True
    
    def _wait_until_stable(self, path):
        """Wait until file size is stable (file completely written)"""
        prev_size = -1
        for attempt in range(1, self.stable_tries + 1):
            if not os.path.exists(path):
                return False
            size = os.path.getsize(path)
            if size == prev_size and size > 0:
                return True
            prev_size = size
            time.sleep(self.stable_wait)
        return False
        
    def on_created(self, event):
        if event.is_directory:
            return
        
        # Only process image files
        if not event.src_path.lower().endswith(('.jpg', '.jpeg', '.png', '.tif')):
            return
        
        # Debounce
        if not self._should_call(event.src_path):
            print(f"[WATCHDOG] Debounced event for {event.src_path}")
            return

        # Wait until file is stable
        if not self._wait_until_stable(event.src_path):
            print(f"[WATCHDOG] File {event.src_path} not stable, skipping")
            return

        print(f"[WATCHDOG] New file detected: {event.src_path}")
        
        # Process in background thread
        threading.Thread(
            target=self._process_new_file, 
            args=(event.src_path,), 
            daemon=True
        ).start()
    
    def _process_new_file(self, file_path):
        """Process a newly detected file: extract metadata, gate it, admit it if it passes."""
        if self.session_id not in sessions:
            return
            
        session = sessions[self.session_id]
        
        # Update image count
        session.image_count = session._count_images()

        meta = extract_flight_metadata(file_path)
        if meta["has_telemetry"]:
            att_ok, worst_angle = session.attitude_filter.should_accept(meta["roll"], meta["pitch"])
            if not att_ok:
                print(f"[FILTER] Rejected (attitude {worst_angle:.1f}° over threshold): {file_path}")
                return

            gps_ok, dist = session.gps_filter.should_accept(meta["latitude"], meta["longitude"])
            if not gps_ok:
                print(f"[FILTER] Rejected (too close, {dist:.1f}m from last accepted): {file_path}")
                return
        else:
            print(f"[FILTER] Accepted WITHOUT telemetry (degraded confidence): {file_path}")

        session.accepted_images.append(Path(file_path))
        session.accepted_metadata.append(meta)
        
        print(f"[WATCHDOG] Session {self.session_id}: {len(session.accepted_images)} total accepted images")
        
        # Notify clients
        asyncio.run(self._notify_clients(session, file_path))
        
        # Check if auto-stitch should trigger
        if session.config.auto_stitch_enabled:
            images_since_last_stitch = len(session.accepted_images) - session.last_stitch_count

            if images_since_last_stitch >= session.config.auto_stitch_threshold and session.claim_stitch():
                print(f"[AUTO-STITCH] Triggering for session {self.session_id} ({images_since_last_stitch} new accepted images)")

                # Trigger stitching
                threading.Thread(
                    target=lambda: asyncio.run(run_stitching(session.session_id, claimed=True)),
                    daemon=True
                ).start()
    
    async def _notify_clients(self, session, file_path):
        """Notify WebSocket clients about new file"""
        for ws in session.ws_clients:
            try:
                await ws.send_json({
                    "type": "file_detected",
                    "file": os.path.basename(file_path),
                    "total_images": session.image_count
                })
            except:
                pass

# Global sessions storage
sessions: Dict[str, StitchingSession] = {}
observers: Dict[str, Observer] = {}

# Folder monitoring functions
def start_folder_monitoring(session_id: str):
    """Start watchdog observer for a session"""
    if session_id not in sessions:
        return False
    
    session = sessions[session_id]
    
    # Stop existing observer if any
    if session_id in observers:
        observers[session_id].stop()
        observers[session_id].join()
    
    # Create new observer
    event_handler = SessionFolderHandler(session_id)
    observer = Observer()
    observer.schedule(event_handler, str(session.image_folder), recursive=False)
    observer.start()
    
    observers[session_id] = observer
    session.observer = observer
    
    print(f"[WATCHDOG] Started monitoring: {session.image_folder}")
    return True

def stop_folder_monitoring(session_id: str):
    """Stop watchdog observer for a session"""
    if session_id in observers:
        observers[session_id].stop()
        observers[session_id].join()
        del observers[session_id]
        print(f"[WATCHDOG] Stopped monitoring for session {session_id}")
        return True
    return False

def discover_existing_sessions():
    """
    Set up sessions from config.UAV_CONFIG first (monitoring/auto-stitch as
    configured there -- single source of truth shared with receiver_socket.py),
    then pick up any other pre-existing session folders not covered by the
    config (monitoring off by default, matching prior behavior for ad-hoc/manual
    sessions like leftover test folders).
    """
    sessions_root = Path("./sessions")
    sessions_root.mkdir(parents=True, exist_ok=True)

    print("[STARTUP] Setting up configured UAV sessions...")
    for uav_cfg in config.UAV_CONFIG.values():
        session_id = uav_cfg["session_id"]
        cfg = StitchConfig(
            sessionId=session_id,
            auto_stitch_threshold=uav_cfg["auto_stitch_threshold"],
            auto_stitch_enabled=uav_cfg["auto_stitch_enabled"],
            folder_monitoring_enabled=uav_cfg["folder_monitoring_enabled"],
            output_name=uav_cfg["output_name"],
        )
        sessions[session_id] = StitchingSession(session_id, cfg)
        if cfg.folder_monitoring_enabled:
            start_folder_monitoring(session_id)
        print(f"[STARTUP]   - {session_id}: monitoring={cfg.folder_monitoring_enabled}, auto_stitch={cfg.auto_stitch_enabled}")

    print("[STARTUP] Discovering other existing session folders...")
    discovered = 0

    for session_dir in sessions_root.iterdir():
        if not session_dir.is_dir():
            continue

        session_id = session_dir.name
        if session_id in sessions:
            continue  # already set up from config.UAV_CONFIG above

        images_dir = session_dir / "images"

        # Check if it looks like a valid session
        if images_dir.exists():
            print(f"[STARTUP] Found session: {session_id}")

            # Create session with default config (monitoring off -- ad-hoc/manual session)
            cfg = StitchConfig(
                sessionId=session_id,
                auto_stitch_threshold=5,
                auto_stitch_enabled=False,
                folder_monitoring_enabled=False,
                output_name="finalResult.png"
            )

            sessions[session_id] = StitchingSession(session_id, cfg)
            print(f"[STARTUP]   - {sessions[session_id].image_count} images found")
            discovered += 1

    print(f"[STARTUP] Discovered {discovered} additional session(s)")

# Stitching function using your existing code
async def run_stitching(session_id: str, claimed: bool = False):
    """Run the stitching process using your ImageMosaic logic using the accepted filtered image list"""
    session = sessions[session_id]
    if not claimed and not session.claim_stitch():
        return
    
    # Notify clients that stitching started
    for ws in session.ws_clients:
        try:
            await ws.send_json({
                "type": "stitching_started",
                "image_count": session.image_count
            })
        except:
            pass
    
    start_time = time.time()
    success = False
    error_msg = None
    
    try:
        images_to_stitch = list(session.accepted_images)
        metadata_to_stitch = list(session.accepted_metadata)
        print(f"[STITCH] Loading {len(images_to_stitch)} accepted images from {session.image_folder}")
        
        images = []
        valid_metadata = []
        for path, meta in zip(images_to_stitch, metadata_to_stitch):
            img = cv2.imread(str(path))
            if img is not None:
                images.append(img)
                valid_metadata.append(meta)
            else:
                print(f"[STITCH] Warning: failed to read {path}")

        if len(images) < 2:
            raise Exception("Not enough valid accepted images for stitching")

        dataMatrix = build_data_matrix(valid_metadata)

        print("[STITCH] Starting mosaic generation...")
        my_combiner = Combiner.Combiner(images, dataMatrix, str(session.output_folder))
        result = my_combiner.create_mosaic()

        if result is not None:
            output_path = session.output_folder / session.config.output_name
            cv2.imwrite(str(output_path), result)
            print(f"[STITCH] Saved result to {output_path}")
            success = True
        else:
            error_msg = "Stitching failed"

    except Exception as e:
        import traceback
        print(f"[STITCH] Error: {e}")
        traceback.print_exc()
        error_msg = str(e)

    elapsed_time = time.time() - start_time
    
    # Notify clients that stitching completed
    for ws in session.ws_clients:
        try:
            await ws.send_json({
                "type": "stitching_completed",
                "success": success,
                "elapsed_time": elapsed_time,
                "error_message": error_msg,
                "output_file": f"/session/{session_id}/result" if success else None
            })
        except:
            pass
    
    session.finish_stitch()

# Lifespan context manager for startup and shutdown
@asynccontextmanager
async def lifespan(app: FastAPI):
    # Startup
    print("[STARTUP] Live Orthomosaic Stitcher starting...")
    discover_existing_sessions()
    yield
    # Shutdown
    print("[SHUTDOWN] Stopping all monitors...")
    for session_id in list(observers.keys()):
        stop_folder_monitoring(session_id)
    print("[SHUTDOWN] Cleanup complete")

# FastAPI app with lifespan
app = FastAPI(title="Live Orthomosaic Stitcher", lifespan=lifespan)

app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

@app.get("/")
async def root():
    return {
        "service": "Live Orthomosaic Stitcher",
        "version": "1.0",
        "active_sessions": len(sessions),
        "monitoring": len(observers)
    }

@app.get("/sessions")
async def list_sessions():
    """List all available sessions"""
    return {
        "sessions": [
            {
                "session_id": sid,
                "image_count": session.image_count,
                "is_stitching": session.is_stitching,
                "monitoring": session.config.folder_monitoring_enabled,
                "auto_stitch": session.config.auto_stitch_enabled
            }
            for sid, session in sessions.items()
        ]
    }

@app.post("/session/create")
async def create_session(config: StitchConfig):
    session_id = config.sessionId
    
    if session_id in sessions:
        return {"status": "Session already exists", "session_id": session_id}
    
    sessions[session_id] = StitchingSession(session_id, config)
    
    # Start folder monitoring if enabled
    if config.folder_monitoring_enabled:
        start_folder_monitoring(session_id)
    
    return {
        "status": "Session created",
        "session_id": session_id,
        "image_count": sessions[session_id].image_count,
        "folder_monitoring": config.folder_monitoring_enabled
    }

@app.post("/session/{session_id}/toggle-monitoring")
async def toggle_monitoring(session_id: str, enable: bool):
    """Enable/disable folder monitoring"""
    if session_id not in sessions:
        return {"status": "Session not found"}
    
    session = sessions[session_id]
    session.config.folder_monitoring_enabled = enable
    
    if enable:
        success = start_folder_monitoring(session_id)
        return {"status": "Monitoring enabled" if success else "Failed"}
    else:
        success = stop_folder_monitoring(session_id)
        return {"status": "Monitoring disabled"}

@app.post("/session/{session_id}/toggle-auto-stitch")
async def toggle_auto_stitch(session_id: str, enable: bool):
    """Enable/disable auto-stitching"""
    if session_id not in sessions:
        return {"status": "Session not found"}
    
    session = sessions[session_id]
    session.config.auto_stitch_enabled = enable
    
    return {
        "status": f"Auto-stitch {'enabled' if enable else 'disabled'}",
        "threshold": session.config.auto_stitch_threshold
    }

@app.post("/session/{session_id}/upload")
async def upload_image(session_id: str, files: List[UploadFile]):
    if session_id not in sessions:
        return {"status": "Session not found"}
    
    session = sessions[session_id]
    uploaded = 0
    
    for file in files:
        file_path = session.image_folder / file.filename
        with open(file_path, "wb") as f:
            content = await file.read()
            f.write(content)
        uploaded += 1
        print(f"[UPLOAD] Saved {file.filename} to {session.image_folder}")
    
    # Update count
    session.image_count = session._count_images()
    
    return {
        "uploaded": uploaded,
        "total": session.image_count,
        "session_id": session_id
    }

@app.post("/session/{session_id}/stitch")
async def trigger_stitch(session_id: str, background_tasks: BackgroundTasks):
    if session_id not in sessions:
        return {"status": "Session not found"}
    
    session = sessions[session_id]
    
    if not session.claim_stitch():
        return {"status": "Stitching already in progress"}
    
    # session.last_stitch_count = session.image_count
    background_tasks.add_task(run_stitching, session_id, True)
    return {"status": "Stitching started", "image_count": len(session.accepted_images)}

@app.get("/session/{session_id}/result")
async def get_result_image(session_id: str):
    if session_id not in sessions:
        return {"status": "Session not found"}
    
    session = sessions[session_id]
    output_file = session.output_folder / session.config.output_name
    
    if not output_file.exists():
        # Try intermediate results
        output_files = list(session.output_folder.glob("intermediateResult_*.png"))
        if output_files:
            output_file = max(output_files, key=lambda p: p.stat().st_mtime)
        else:
            return {"status": "No result image found"}
    
    return FileResponse(output_file)

@app.get("/session/{session_id}/intermediates")
async def list_intermediate_results(session_id: str):
    """List all intermediate result images"""
    if session_id not in sessions:
        return {"status": "Session not found"}
    
    session = sessions[session_id]
    intermediates = sorted(
        session.output_folder.glob("intermediateResult_*.png"),
        key=lambda p: p.stat().st_mtime
    )
    
    return {
        "session_id": session_id,
        "count": len(intermediates),
        "files": [f.name for f in intermediates]
    }

@app.get("/session/{session_id}/status")
async def get_session_status(session_id: str):
    if session_id not in sessions:
        return {"status": "Session not found"}
    
    session = sessions[session_id]
    return {
        "session_id": session_id,
        "image_count": session.image_count,
        "accepted_count": len(session.accepted_images),
        "is_stitching": session.is_stitching,
        "auto_stitch_enabled": session.config.auto_stitch_enabled,
        "auto_stitch_threshold": session.config.auto_stitch_threshold,
        "folder_monitoring_enabled": session.config.folder_monitoring_enabled,
        "last_stitch_count": session.last_stitch_count,
        "images_since_last_stitch": len(session.accepted_images) - session.last_stitch_count
    }

@app.websocket("/ws/{session_id}")
async def websocket_endpoint(websocket: WebSocket, session_id: str):
    await websocket.accept()
    
    if session_id not in sessions:
        await websocket.send_json({"error": "Session not found"})
        await websocket.close()
        return
    
    session = sessions[session_id]
    session.ws_clients.append(websocket)
    
    try:
        while True:
            await websocket.receive_text()
    except:
        if websocket in session.ws_clients:
            session.ws_clients.remove(websocket)

if __name__ == "__main__":
    import uvicorn
    uvicorn.run(app, host="127.0.0.1", port=8001)