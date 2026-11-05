"""Replay a fixed 12-image sample without modifying existing results."""
from pathlib import Path
import json
import sys
import time
import cv2
import numpy as np
ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))
from live_stitcher.config import StitchConfig
from live_stitcher.stitcher import LiveStitcher
from live_stitcher.gps_utils import extract_gps_exif
OUT = Path(__file__).parent
files = sorted(p for p in (ROOT/'dataset/test_orthomosaics_42imgs').iterdir()
               if p.suffix.lower() in {'.jpg', '.jpeg', '.png'})[:12]
cfg = StitchConfig(output_dir=str(OUT/'replay_output'))
st = LiveStitcher(cfg)
rows = []
for p in files:
    t = time.perf_counter()
    try:
        result = st.process_file(str(p))
        status = 'mosaic_returned' if result is not None else 'skipped_or_failed'
        error = None
    except Exception as exc:
        status, error = 'exception', str(exc)
    rows.append({'file': p.name, 'status': status, 'error': error,
                 'wall_ms': (time.perf_counter()-t)*1000})
st.save_mosaic('audit_mosaic.png')
mosaic = st.get_mosaic()
scale = min(1, 1200/max(mosaic.shape[:2]))
cv2.imwrite(str(OUT/'replay_preview.jpg'), cv2.resize(mosaic, None, fx=scale, fy=scale))
report = {'dataset': 'dataset/test_orthomosaics_42imgs', 'config': cfg.to_dict(),
          'frames': rows, 'stats': st.get_stats(), 'wall_ms_p50':float(np.median([r['wall_ms'] for r in rows])),
          'wall_ms_p95':float(np.percentile([r['wall_ms'] for r in rows],95)),
          'actual_mosaic_returns': sum(r['status']=='mosaic_returned' for r in rows),
          'canvas_buffers_bytes':st.canvas.color_acc.nbytes+st.canvas.weight_acc.nbytes}
(OUT/'replay.json').write_text(json.dumps(report, indent=2), encoding='utf-8')
print(json.dumps(report, indent=2))
