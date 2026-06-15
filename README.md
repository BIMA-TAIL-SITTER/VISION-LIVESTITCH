
# 🚁 VISION-LIVESTITCH: Aerial Live Image Stitching

VISION-LIVESTITCH is a project focused on creating orthomosaics or performing continuous, real-time aerial image stitching. This project is designed to be computationally efficient, making it highly suitable for live transmission between an edge node (drone) and a Ground Control Station (GCS).

The development of this project is geared towards ultimately serving as the backend microservices for a custom, hand-made GCS in the future.

---

## ✨ Key Features & Architecture

### 1. Feature Detection & Extraction
- Utilizes the **SIFT (Scale-Invariant Feature Transform)** algorithm for local frame-to-frame feature detection and extraction to ensure high accuracy when pairing image iterations.

### 2. Flexible Geometric Transformation (`Combiner.py`)
- By default, it relies on **Affine Transformation** for fast and constant camera angle-change computations between frames.
- **Fallback Mechanism:** If the Affine matrix fails, the system falls back to a *Localized Improved Homography Estimation* approach.
- **Chained Matrix Multiplication:** Rather than utilizing heavy and time-consuming global optimization techniques (such as bundle adjustment), this program maintains geometric coherence and global tracking by multiplying transformation matrices chronologically (chained approach).

### 3. Localized Feather Blending (`blending.py`)
- Implements **Localized Frame-to-Frame Feather Blending**, meaning the transition blending (ROI Feather) is calculated exclusively over the overlapping regions between local frames.
- The main advantage is a drastic reduction in computational load, dropping the need for global blending arrays to support true real-time capabilities.

### 4. Optional Geometry Correction (`geometry.py` & `utilities.py`)
- **Unrotation Matrix:** This module includes an unrotation matrix computation to correct pitch, roll, and yaw. This can be enabled if the hardware orientation data comes complete with IMU (telemetry) data. If IMU data is absent, the visual frame-to-frame algorithmic computation will act as the primary structural compass.
- **Warp Perspective & Padding:** Preserves the scale integrity by dynamically adjusting the coordinate boundary canvas (width × height) during each warp iteration.
- Various additional functions for GPS coordinate extraction (via EXIF metadata) are also included in the utilities module.

### 5. Microservices Architecture & Live Transmission (`service.py`)
This project introduces a prototype for a microservices architecture:
- **Backend API & Watcher (`service.py`):** Responsible for monitoring incoming image availability and automatically stitching newly arrived images continuously via Threads / Background Tasks (powered by FastAPI).
- **Socket Receiver (`receiver_socket.py`):** A listener interface (Socket) that receives incoming image packets from the airborne edge device.
- **Simulated Socket Sender (`sender_socket_sim.py`):** A dummy/gimmick sender program that simulates continuous image transmission from the drone (edge) to the GCS application.

---

## ⚙️ System Requirements

All required packages and environment modules are listed in the `requirements.txt` file. The core dependencies include (but are not limited to):
- `numpy`, `opencv-python`, `scikit-image` for matrix and image computation.
- `FastAPI`, `uvicorn`, `websockets` for the microservices framework and real-time connectivity.
- `exifread`, `scipy` for reading metadata and additional spatial computation.
- Other utility tools (e.g., `watchdog`).

To install all dependencies, run the following command in your virtual environment:
- Activate/create your virtual environment first
```bash
./setup_venv.sh
```
- or if you already have your virtual environment, run this command:
```bash
/.activate_venv.sh
```
- Then install the required package by running this command:

```bash
pip install -r requirements.txt
```

---

## 🚀 Getting Started

You do not need to boot up each component manually. The entire system can be executed via a single, concise shell script.

1. **Open your Terminal** and ensure you are in the root directory of the repository (`VISION-LIVESTITCH`).
2. **Grant execution permissions (if you haven't already):**
   ```bash
   chmod +x LIVE_STITCHING_START.sh
   ```
3. **Run the Automation Script:**
   ```bash
   ./LIVE_STITCHING_START.sh
   ```
   *(Optional: You can provide a session name argument, e.g., `.LIVE_STITCHING_START.sh my_session`)*

The script above will automatically spawn 2 separate terminals:
1. **Service Terminal:** Initializes the web API and the core stitching logic (auto-stitch watcher).
2. **Receiver Terminal:** Listens for TCP/Socket image transmissions coming from (or simulated as) a flying drone on port `5001`.

**For Transmission Testing (Drone Simulation):**
Open a 3rd terminal and execute the sender script:
```bash
python sender_socket_sim.py --dataset-dir <YOUR_DATASET_FOLDER_NAME>
```
Images will automatically be dispatched periodically, caught by the receiver, intercepted by the service watcher, and stitched completely in real-time!

---

## 🗺️ Roadmap
- Seamless integration of this backend with a standalone Ground Control Station (GCS) user interface (Frontend).
- Further algorithm optimization to broaden the capability of keypoint distributions for large-scale flight operations.
- Optimization for multi-UAV or Swarm Flight operations image stitching
- Optimize microservice architecture and introduce containerization (e.g., Docker) to streamline future GCS deployments.