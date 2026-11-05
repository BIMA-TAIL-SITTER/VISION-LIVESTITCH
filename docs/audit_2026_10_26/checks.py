"""Bounded, offline audit probes. Does not edit production code or use hardware."""
import ast
import contextlib
import hashlib
import importlib.util
import io
import json
import os
from pathlib import Path
import struct
import sys
import tempfile

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))
os.chdir(ROOT)
import cv2
import numpy as np

OUT = Path(__file__).parent
results = {}
inventory = []
skip = {'.git', '.venv', 'venv', '__pycache__', 'dataset', 'sessions', 'audit_2026_10_26'}
for directory, dirs, files in os.walk(ROOT):
    dirs[:] = [d for d in dirs if d not in skip]
    for name in files:
        if not name.endswith('.py'):
            continue
        path = Path(directory) / name
        source = path.read_text(encoding='utf-8-sig')
        entry = {'file': path.relative_to(ROOT).as_posix(), 'lines': len(source.splitlines()),
                 'sha256': hashlib.sha256(source.encode()).hexdigest()}
        try:
            tree = ast.parse(source)
            entry['syntax'] = 'OK'
            entry['symbols'] = [f'{n.name}:{n.lineno}' for n in ast.walk(tree)
                                if isinstance(n, (ast.FunctionDef, ast.AsyncFunctionDef, ast.ClassDef))]
        except SyntaxError as exc:
            entry['syntax'] = str(exc)
        inventory.append(entry)
results['inventory'] = {'python_files': len(inventory), 'lines': sum(x['lines'] for x in inventory),
                        'syntax_errors': [x for x in inventory if x['syntax'] != 'OK']}

from live_stitcher.config import StitchConfig
from live_stitcher.registrator import Registrator
from live_stitcher.canvas import IncrementalCanvas
from live_stitcher.stitcher import LiveStitcher
from src.geometry import computeUnRotMatrix

# Exact correspondences with a real scale change: the reported inlier mask
# is computed BEFORE normalization to rigid, so check the returned transform.
rng = np.random.default_rng(42)
src = rng.uniform(50, 500, (40, 2)).astype(np.float32)
dst = src * 1.1 + np.array([20, -10], dtype=np.float32)
H, mask = Registrator(StitchConfig()).estimate_transform(src, dst, [(i, i) for i in range(40)])
pred = cv2.perspectiveTransform(src[:, None, :], H)[:, 0, :]
errors = np.linalg.norm(pred-dst, axis=1)
results['rigid_postfit'] = {'reported_inliers': int(mask.sum()),
                          'median_error_px': float(np.median(errors)),
                          'max_error_px': float(errors.max())}

# First-image content validity and dark real pixels on subsequently added frame.
cfg = StitchConfig(canvas_initial_padding=0, canvas_expand_margin=0,
                   enable_exposure_compensation=False, enable_multiband_blend=False)
c = IncrementalCanvas(cfg)
c.initialize(np.full((40, 40, 3), 100, np.uint8))
c.add_image(np.zeros((40, 40, 3), np.uint8), np.array([[1, 0, 40], [0, 1, 0], [0, 0, 1.]], float))
results['black_pixel_mask'] = {'expected_new_valid_pixels': 1600,
                             'actual_new_valid_pixels': int((c.weight_acc[:, 40:, 0] > 0).sum())}

with contextlib.nullcontext(str(OUT / 'probe_live')) as td:
    st = LiveStitcher(StitchConfig(output_dir=td, downsample_factor=1, enable_heading_correction=False,
                                  canvas_initial_padding=0))
    image = rng.integers(1, 255, (180, 180, 3), dtype=np.uint8)
    a = {'latitude': -7.0, 'longitude': 112.0, 'altitude': 80}
    b = {'latitude': -7.0, 'longitude': 112.00005, 'altitude': 80}
    st.process_frame(image, a)
    failed = st.process_frame(np.zeros_like(image), b)
    retry = st.process_frame(image, b)
    results['filter_committed_before_registration'] = {'failed_is_none': failed is None,
        'good_retry_is_rejected': retry is None, 'stats': st.get_stats(),
        'last_successful_gps': st._prev_gps, 'filter_position': st.filter.last_position}

def load(name, path):
    spec = importlib.util.spec_from_file_location(name, path)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod

sys.path.insert(0, str(ROOT / 'PROGRAM_JUANG/PROGRAM-SENDER1'))
receiver = load('audit_udp_receiver', ROOT / 'PROGRAM_JUANG/PROGRAM-SENDER1/receiver.py')
asm = receiver.FrameReassembler()
asm.feed(struct.pack('!III', 1, 2, 0) + b'UAV_A_0')
mixed = asm.feed(struct.pack('!III', 1, 2, 1) + b'UAV_B_1')
results['udp_source_collision'] = {'assembled': mixed.decode(), 'sources_distinguished': False}
asm = receiver.FrameReassembler()
try:
    asm.feed(struct.pack('!III', 2, 1, 999) + b'bad-index')
    results['udp_invalid_index'] = 'no exception'
except Exception as exc:
    results['udp_invalid_index'] = type(exc).__name__ + ': ' + str(exc)

ground = load('audit_ground_stitcher', ROOT / 'PROGRAM_JUANG/PROGRAM-SENDER1/stitcher.py')
with contextlib.nullcontext(str(OUT / 'probe_batch')) as td:
    mon = ground.SessionMonitor('test', td, 2, 3)
    files = []
    for i in range(2):
        path = Path(td) / f'img{i}.jpg'
        cv2.imwrite(str(path), np.full((32, 32, 3), 100+i, np.uint8))
        files.append(path)
    mon.process_new_images(files)
    mon.engine.stitch = lambda images, metadata: images[0]
    before = mon.should_stitch()
    mon.run_stitch()
    results['batch_without_new_frames'] = {'ready_before': before, 'ready_after': mon.should_stitch(),
                                         'generated_metadata': mon._accepted_gps}

H_degrees = computeUnRotMatrix(np.array([0, 0, 0, 90, 0, 0]))
H_radians = computeUnRotMatrix(np.array([0, 0, 0, np.pi/2, 0, 0]))
results['angle_unit_mismatch'] = {'expected_deg': float(np.degrees(np.arctan2(H_degrees[1,0],H_degrees[0,0]))),
                                'actual_deg': float(np.degrees(np.arctan2(H_radians[1,0],H_radians[0,0])))}
for mod in ['service', 'core_gmf.image_corrector']:
    try:
        __import__(mod)
        results['import_' + mod] = 'OK'
    except Exception as exc:
        results['import_' + mod] = type(exc).__name__ + ': ' + str(exc)

(OUT / 'inventory.json').write_text(json.dumps(inventory, indent=2), encoding='utf-8')
(OUT / 'results.json').write_text(json.dumps(results, indent=2), encoding='utf-8')
print(json.dumps(results, indent=2))
