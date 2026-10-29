"""Calibrate a fixed camera mode from >=10 chessboard observations."""
from pathlib import Path
import cv2
import numpy as np
from .geometry import Calibration
from .storage import write_json


def run(args):
    if args.cols < 3 or args.rows < 3 or args.square_mm <= 0:
        raise ValueError("invalid chessboard dimensions")
    grid = np.zeros((args.rows*args.cols,3),np.float32)
    grid[:,:2] = np.mgrid[0:args.cols,0:args.rows].T.reshape(-1,2)*args.square_mm
    object_points,image_points = [],[]
    size = None
    used = []
    for path in sorted(Path(args.images).iterdir()):
        if path.suffix.lower() not in {".jpg",".jpeg",".png"}:
            continue
        image = cv2.imread(str(path),cv2.IMREAD_GRAYSCALE)
        if image is None:
            continue
        current = (image.shape[1],image.shape[0])
        if size is not None and current != size:
            raise ValueError("all calibration photos must use one resolution/camera mode")
        size = current
        found,corners = cv2.findChessboardCorners(image,(args.cols,args.rows))
        if found:
            corners = cv2.cornerSubPix(image,corners,(11,11),(-1,-1),
                (cv2.TERM_CRITERIA_EPS+cv2.TERM_CRITERIA_MAX_ITER,30,0.001))
            object_points.append(grid.copy())
            image_points.append(corners)
            used.append(path.name)
    if len(used) < 10:
        raise ValueError(f"only {len(used)} usable chessboards; at least 10 needed")
    rms,K,dist,rvecs,tvecs = cv2.calibrateCamera(object_points,image_points,size,None,None)
    cal = Calibration(calibration_id=args.calibration_id,camera_id=args.camera_id,
        image_width=size[0],image_height=size[1],camera_matrix=K.tolist(),
        distortion=dist.ravel().tolist(),rms_error_px=rms)
    write_json(args.output,cal.model_dump(mode="json"))
    errors = []
    for xyz,observed,r,t in zip(object_points,image_points,rvecs,tvecs):
        projected,_ = cv2.projectPoints(xyz,r,t,K,dist)
        errors.append(float(np.sqrt(np.mean(np.sum((observed-projected)**2,axis=2)))))
    write_json(str(args.output)+".report.json",{"rms_px":rms,"files":used,"per_image_rms_px":errors})
    print(f"Saved calibration: {args.output}; RMS={rms:.3f}px. Validate on held-out images before flight.")
