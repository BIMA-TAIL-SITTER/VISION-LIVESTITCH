# PROGRAM-NOVA – Live Stitching UAV Fixed Wing

Sistem live stitching gambar udara untuk UAV **Fixed Wing** menggunakan:
- **Kamera**: DJI Osmo Action 5 Pro (mode UVC Webcam via USB)
- **Onboard Computer**: Raspberry Pi 5 16GB
- **Flight Controller**: ArduPilot / PX4 (terhubung via UART/MAVLink)
- **Ground Station**: Laptop dengan Python

---

## Arsitektur Sistem

```
┌─────────────────────────────────────────────────┐
│                  UAV (Fixed Wing)               │
│                                                 │
│  ┌──────────────┐     USB/UVC    ┌───────────┐  │
│  │ DJI Osmo     │─────────────▶│           │  │
│  │ Action 5 Pro │               │ Raspberry │  │
│  │ (Webcam mode)│               │   Pi 5    │  │
│  └──────────────┘               │           │  │
│                                 │ sender.py │  │
│  ┌──────────────┐    UART/      │           │  │
│  │   Flight     │─ MAVLink ──▶ │ imu_mon.  │  │
│  │ Controller   │               │           │  │
│  │(Ardupilot/   │               └─────┬─────┘  │
│  │   PX4)       │                     │         │
│  └──────────────┘              UDP (WiFi/Radio) │
└────────────────────────────────────┼────────────┘
                                     │
                   ┌─────────────────▼─────────────┐
                   │         GROUND STATION         │
                   │                                │
                   │  receiver.py  ──▶  stitcher.py │
                   │       │               │         │
                   │  od_simulator.py   mosaic +     │
                   │  (simulasi OD)    OD overlay    │
                   └────────────────────────────────┘
```

---

## Struktur File

```
PROGRAM-NOVA/
├── sender.py               ← Di-upload ke Raspberry Pi
├── imu_monitor.py          ← Di-upload ke Raspberry Pi (standalone)
├── receiver.py             ← Dijalankan di Ground Station
├── stitcher.py             ← Dijalankan di Ground Station
├── od_simulator.py         ← Simulasi OD (testing)
├── run_ground.py           ← Launcher semua proses ground sekaligus
├── config.py               ← Konfigurasi terpusat (edit IP di sini!)
├── requirements_raspi.txt  ← Install di Raspberry Pi
└── requirements_ground.txt ← Install di Laptop/Ground
```

---

## Pertanyaan 1: Setup DJI Osmo Action 5 Pro sebagai Webcam

### Langkah-langkah:

**A. Di kamera DJI Osmo Action 5 Pro:**
1. Masuk ke **Settings → General → USB Mode**
2. Pilih **"UVC" atau "Webcam"** (bukan "Storage" / "Charge")
   > Osmo Action 5 Pro mendukung UVC natively – kamera akan terdeteksi sebagai webcam standar
3. Hubungkan ke Raspberry Pi via kabel **USB-C**

**B. Verifikasi di Raspberry Pi:**
```bash
# Cek apakah kamera terdeteksi
ls /dev/video*

# Lihat detail device (harus muncul DJI Action)
v4l2-ctl --list-devices

# Cek resolusi yang didukung
v4l2-ctl -d /dev/video0 --list-formats-ext

# Test capture frame (simpan ke file)
ffmpeg -f v4l2 -i /dev/video0 -frames:v 1 test_capture.jpg
```

**C. Setting di `config.py`:**
```python
CAMERA_INDEX  = 0       # /dev/video0
CAMERA_WIDTH  = 4096    # Bisa dikurangi ke 1920 untuk hemat bandwidth
CAMERA_HEIGHT = 3072    # atau 1080
CAMERA_FPS    = 5       # 5 fps cukup untuk pemetaan
```

### Tips Kamera:
- Gunakan mode **Wide** untuk coverage terluas
- **Aktifkan EIS** (Electronic Image Stabilization) di kamera untuk gambar lebih stabil
- Set **exposure manual** agar brightness konsisten saat terbang
- Format **JPEG** sudah digunakan (bukan RAW) untuk menghemat bandwidth

---

## Pertanyaan 2: Program di Raspberry Pi & Cara Menjalankan

### File yang di-upload ke Raspberry Pi:
```
sender.py
imu_monitor.py      (opsional, sudah terintegrasi di sender.py)
config.py
requirements_raspi.txt
```

### Setup Raspberry Pi:

**1. Install dependencies:**
```bash
# Update sistem
sudo apt update && sudo apt upgrade -y

# Install OpenCV dependencies
sudo apt install -y python3-opencv v4l-utils

# Install Python packages
pip3 install -r requirements_raspi.txt
```

**2. Setup UART untuk FC (jika menggunakan GPIO UART):**
```bash
# Aktifkan UART di Raspberry Pi 5
sudo raspi-config
# → Interface Options → Serial Port
# → "Login shell accessible over serial?" → No
# → "Serial port hardware enabled?" → Yes

# Verifikasi port
ls /dev/ttyAMA*   # atau /dev/ttyUSB0 jika via USB
```

**3. Set permission serial port:**
```bash
sudo usermod -a -G dialout $USER
```

**4. Edit config.py – ganti IP:**
```python
GROUND_IP = "192.168.1.100"   # ← Ganti dengan IP laptop ground station!
```

**5. Jalankan sender:**
```bash
# Mode normal (dengan IMU dari FC)
python3 sender.py

# Mode tanpa FC (untuk testing kamera)
python3 sender.py --no-fc

# Dengan parameter custom
python3 sender.py \
    --host 192.168.1.100 \
    --img-port 5600 \
    --fc-port /dev/ttyAMA0 \
    --fc-baud 57600 \
    --interval 0.5 \
    --quality 85
```

**6. (Opsional) Jalankan sebagai systemd service (auto-start):**
```bash
# Buat file service
sudo nano /etc/systemd/system/nova-sender.service
```
```ini
[Unit]
Description=NOVA UAV Sender
After=network.target

[Service]
ExecStart=/usr/bin/python3 /home/pi/PROGRAM-NOVA/sender.py
WorkingDirectory=/home/pi/PROGRAM-NOVA
Restart=always
RestartSec=5
User=pi

[Install]
WantedBy=multi-user.target
```
```bash
sudo systemctl enable nova-sender
sudo systemctl start nova-sender
sudo systemctl status nova-sender
```

---

## Pertanyaan 3: Membaca IMU dari FC ke Raspberry Pi

### Koneksi Fisik (UART):

```
Flight Controller          Raspberry Pi 5
─────────────────          ──────────────
  TELEM2 TX    ──────────▶  GPIO 15 (RXD) [Pin 10]
  TELEM2 RX    ◀──────────  GPIO 14 (TXD) [Pin 8]
  GND          ──────────▶  GND            [Pin 6]
```

> ⚠️ **PENTING**: FC biasanya bekerja di 3.3V – Raspberry Pi GPIO juga 3.3V, AMAN.
> Jangan gunakan level converter kecuali FC menggunakan 5V logic.

### Setting Flight Controller:

**ArduPilot:**
```
SERIAL2_PROTOCOL = 2      (MAVLink 2)
SERIAL2_BAUD     = 57     (57600 baud)
SERIAL2_OPTIONS  = 0
```

**PX4:**
```
MAV_1_CONFIG   = TELEM2
SER_TELEM2_BAUD = 57600
MAV_1_MODE     = Normal
MAV_1_RATE     = 1200 B/s
```

### Test Koneksi MAVLink:
```bash
# Install pymavlink
pip3 install pymavlink

# Test koneksi
python3 -c "
from pymavlink import mavutil
mav = mavutil.mavlink_connection('/dev/ttyAMA0', baud=57600)
mav.wait_heartbeat()
print('FC terdeteksi!')
msg = mav.recv_match(type='ATTITUDE', blocking=True, timeout=5)
import math
print(f'Roll={math.degrees(msg.roll):.1f} Pitch={math.degrees(msg.pitch):.1f}')
"
```

### Jalankan IMU Monitor standalone:
```bash
python3 imu_monitor.py --port /dev/ttyAMA0 --baud 57600
```

### Cara Kerja IMU Gate di `sender.py`:
```
Pesawat berbelok:
  Roll > 15° ATAU Pitch > 20°
  → Frame DITAHAN, tidak dikirim ke ground
  → Kamera tetap berjalan (buffer dibersihkan)

Pesawat lurus kembali:
  Roll < 15° DAN Pitch < 20° selama 5 frame berturut-turut
  → Streaming DILANJUTKAN
  → Hysteresis mencegah false start saat turbulence singkat
```

---

## Cara Menjalankan di Ground Station

**1. Install dependencies:**
```bash
pip install -r requirements_ground.txt
```

**2. Jalankan semua proses sekaligus:**
```bash
# Mode normal
python3 run_ground.py --session penerbangan_01

# Dengan OD simulator (testing tanpa UAV)
python3 run_ground.py --session test_01 --simulate-od

# Custom parameter
python3 run_ground.py \
    --session penerbangan_01 \
    --batch 5 \
    --gps-thresh 3.0
```

**3. Atau jalankan satu per satu (terminal terpisah):**
```bash
# Terminal 1 – Receiver
python3 receiver.py --session penerbangan_01

# Terminal 2 – Stitcher
python3 stitcher.py --session penerbangan_01 --batch 5

# Terminal 3 – OD Simulator (opsional)
python3 od_simulator.py --host 127.0.0.1 --od-port 5601
```

**4. Hasil output:**
```
sessions/
└── penerbangan_01/
    ├── images/           ← Gambar dari UAV
    ├── output/
    │   ├── mosaic_001_20260803_143000.png   ← Mosaic dengan OD overlay
    │   ├── mosaic_002_...png
    │   └── mosaic_latest.png                ← Selalu mosaic terbaru
    └── detections.jsonl  ← Semua data OD (JSON Lines)
```

---

## Format Paket Object Detection (UDP JSON)

```json
{
    "timestamp": 1722675600.123,
    "source": "od_simulator",
    "latitude": -7.123456,
    "longitude": 112.654321,
    "altitude": 80.5,
    "detections": [
        {
            "label": "person",
            "confidence": 0.92,
            "bbox": [320, 240, 480, 420],
            "geo": {
                "lat": -7.123512,
                "lon": 112.654389,
                "alt": 80.5
            }
        }
    ]
}
```

---

## Saran: Menangani Gambar Saat Fixed Wing Berbelok

### Masalah:
Fixed wing **tidak bisa hover** – saat berbelok (*bank turn*), kamera miring dan gambar tidak ortogonal, sehingga hasil stitching jelek dan tidak akurat.

### Solusi yang Diimplementasikan (IMU Gate):

| Kondisi | Roll | Pitch | Aksi |
|---------|------|-------|------|
| ✅ Lurus (straight & level) | < 15° | < 20° | **Kirim gambar** |
| ⚠️ Berbelok (bank turn) | > 15° | any | **Tahan gambar** |
| ⚠️ Menanjak/menukik | any | > 20° | **Tahan gambar** |
| 🔄 Recover | < 15° AND < 20° selama 5 frame | **Resume** |

### Saran Tambahan untuk Operasi:

**1. Desain misi lintasan lurus (Lawnmower pattern)**
```
──────────────────────────────▶  (lurus, foto aktif)
                               ↓
◀──────────────────────────────  (lurus, foto aktif)
↓
──────────────────────────────▶  (lurus, foto aktif)
```
Di tikungan, IMU gate otomatis menahan foto.

**2. Parameter threshold yang disarankan:**
```python
# Untuk area datar (pemetaan)
ROLL_THRESHOLD_DEG  = 12.0   # Lebih ketat
PITCH_THRESHOLD_DEG = 15.0

# Untuk kondisi angin (lebih toleran)
ROLL_THRESHOLD_DEG  = 20.0
PITCH_THRESHOLD_DEG = 25.0
```

**3. Gunakan GPS Threshold stitcher:**
Sudah diimplementasikan di `stitcher.py` – gambar terlalu dekat (< 3 meter) otomatis dilewati agar stitching lebih cepat.

**4. Altitude yang disarankan:**
- **50-100 meter**: Coverage baik, resolusi tinggi
- Terlalu tinggi (> 150m): Detail berkurang
- Terlalu rendah (< 30m): Overlap terlalu sedikit

---

## Troubleshooting

| Problem | Solusi |
|---------|--------|
| Kamera tidak terdeteksi | `v4l2-ctl --list-devices`, pastikan USB mode = UVC |
| FC tidak merespons | Cek kabel UART, `SERIAL2_PROTOCOL=2` di ArduPilot |
| Gambar tidak terkirim | Cek IP di `config.py`, cek firewall, cek port UDP |
| Stitching gagal | Kurangi batch size, pastikan overlap gambar cukup |
| OD tidak muncul di mosaic | Cek `detections.jsonl`, pastikan GPS referensi benar |

---

## Konfigurasi Jaringan

```
Opsi 1: WiFi Lokal (jarak dekat ≤ 300m)
  UAV (Raspi) ──WiFi──▶ Router ──▶ Laptop Ground

Opsi 2: Tailscale VPN (jarak jauh, via internet)
  UAV ──4G──▶ Tailscale ──▶ Laptop Ground
  Ganti GROUND_IP di config.py dengan IP Tailscale

Opsi 3: Radio Telemetry (SiK Radio 915MHz)
  Untuk telemetry MAVLink saja (bukan stream gambar)
  Gambar tetap via WiFi
```
