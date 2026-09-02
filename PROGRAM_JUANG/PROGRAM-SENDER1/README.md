# PROGRAM-SENDER – Live Stitching UAV Fixed Wing

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
PROGRAM-SENDER/
├── sender.py               ← Di-upload ke Raspberry Pi
├── imu_monitor.py          ← Di-upload ke Raspberry Pi (standalone)
├── fc_router.py            ← Opsional: MAVLink UDP fan-out (1 FC ke banyak listener)
├── imu_simulator.py        ← FC palsu via MAVLink UDP (testing tanpa hardware)
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
sudo nano /etc/systemd/system/uav-sender.service
```
```ini
[Unit]
Description=UAV Image Sender
After=network.target

[Service]
ExecStart=/usr/bin/python3 /home/pi/PROGRAM-SENDER/sender.py
WorkingDirectory=/home/pi/PROGRAM-SENDER
Restart=always
RestartSec=5
User=pi

[Install]
WantedBy=multi-user.target
```
```bash
sudo systemctl enable uav-sender
sudo systemctl start uav-sender
sudo systemctl status uav-sender
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
  Roll > 45° ATAU Pitch > 45°
  → Frame DITAHAN, tidak dikirim ke ground
  → Kamera tetap berjalan (buffer dibersihkan)

Pesawat lurus kembali:
  Roll < 45° DAN Pitch < 45° selama 5 frame berturut-turut
  → Streaming DILANJUTKAN
  → Hysteresis mencegah false start saat turbulence singkat
```

---

## Testing dengan FC CUAV Asli (di Laptop, sebelum ke Raspi)

FC CUAV (berbasis ArduPilot) bisa langsung dites di laptop via kabel **USB**, tanpa Raspberry Pi dulu — port USB-nya sudah otomatis jadi virtual serial MAVLink, jadi nggak perlu setting `SERIAL2_PROTOCOL` segala (itu cuma untuk port TELEM/UART, bukan USB).

**1. Colok CUAV FC ke laptop via USB-C/micro-USB, lalu cek portnya muncul dimana:**
```bash
dmesg | tail -20          # cari baris "cdc_acm" atau "ttyACM"
ls /dev/ttyACM*           # biasanya /dev/ttyACM0
```

**2. Kasih izin akses serial port (sekali aja, lalu logout/login ulang):**
```bash
sudo usermod -a -G dialout $USER
```

**3. Test koneksi mentah dulu (belum pakai program NOVA-SENDER):**
```bash
python3 -c "
from pymavlink import mavutil
mav = mavutil.mavlink_connection('/dev/ttyACM0', baud=115200)
mav.wait_heartbeat()
print('FC CUAV terdeteksi!')
"
```
> Baud rate via USB biasanya **115200**, beda dengan UART/TELEM yang 57600. Kalau gagal connect, coba baud lain atau cek `ls /dev/ttyACM*` lagi setelah dicolok ulang.

**4. Kalau sudah konek, jalankan `imu_monitor.py` langsung ke FC beneran:**
```bash
python3 imu_monitor.py --port /dev/ttyACM0 --baud 115200 --no-send
```
Sekarang **miringkan fisik FC-nya** (bukan simulasi lagi) — begitu roll/pitch lewat 45°, status di terminal harus berubah jadi `🔴 PESAWAT BERBELOK`. FC nggak perlu di-arm atau ada GPS fix buat ini; ATTITUDE stream jalan begitu IMU aktif (biasanya langsung nyala pas dicolok).

**5. Kalau mau full test dengan kamera juga (`sender.py`), sama, tinggal ganti `--fc-port`:**
```bash
python3 sender.py --fc-port /dev/ttyACM0 --fc-baud 115200 --host 127.0.0.1 --cam-index 0
```

**6. Pindah ke Raspberry Pi (real flight):**
Begitu sudah yakin gate-nya bener, tinggal pindahin fisik FC ke Raspi via UART (lihat bagian "Koneksi Fisik (UART)" di atas), lalu jalankan dengan `--fc-port /dev/ttyAMA0 --fc-baud 57600` — kode-nya sama persis, cuma port & baud yang beda.

> **Opsional — lebih realistis lagi:** kalau nanti butuh simulasi yang bener-bener meniru fisika pesawat (bukan cuma pola roll/pitch buatan seperti `imu_simulator.py`), ArduPilot punya **SITL** (Software In The Loop) yang bisa jalan di laptop dan nge-stream MAVLink via UDP persis kayak FC asli — bisa dites juga karena `sender.py`/`imu_monitor.py` udah support koneksi UDP. Kabarin aja kalau mau dibantu setup itu.

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

### Solusi yang Diimplementasikan (IMU Gate — 2 lapis):

**Lapis 1 — Onboard (`sender.py`, saat capture di UAV):**

| Kondisi | Roll | Pitch | Aksi |
|---------|------|-------|------|
| ✅ Lurus (straight & level) | < 45° | < 45° | **Kirim gambar** |
| ⚠️ Berbelok (bank turn) | > 45° | any | **Tahan gambar** |
| ⚠️ Menanjak/menukik | any | > 45° | **Tahan gambar** |
| 🔄 Recover | < 45° AND < 45° selama 5 frame | **Resume** |

**Lapis 2 — Ground (`stitcher.py`, sebelum stitching):**
Gambar yang lolos gate onboard tetap disaring ULANG di `stitcher.py` berdasarkan attitude (roll/pitch) yang tersemat di EXIF-nya (`AttitudeThresholdFilter`, default **30°**, diatur via `STITCH_ATTITUDE_THRESHOLD_DEG` di `config.py` atau flag `--attitude-threshold`). Ini gate independen — standar kualitas mosaic bisa diperketat/diperlonggar tanpa redeploy `sender.py` ke UAV.

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
# Onboard (sender.py) — untuk area datar (pemetaan)
ROLL_THRESHOLD_DEG  = 30.0   # Lebih ketat dari default 45°
PITCH_THRESHOLD_DEG = 30.0

# Onboard — untuk kondisi angin (lebih toleran)
ROLL_THRESHOLD_DEG  = 45.0
PITCH_THRESHOLD_DEG = 45.0

# Ground (stitcher.py) — biasanya disetel sedikit LEBIH KETAT dari onboard,
# karena ini adalah gate kualitas terakhir sebelum gambar benar-benar dipakai
STITCH_ATTITUDE_THRESHOLD_DEG = 30.0
```

**3. Gunakan GPS + Attitude Threshold stitcher:**
Sudah diimplementasikan di `stitcher.py` — gambar terlalu dekat (< 3 meter) otomatis dilewati agar stitching lebih cepat (`GPSThresholdFilter`), DAN gambar dengan attitude (roll/pitch) di atas ambang batas otomatis ditolak dari batch stitching (`AttitudeThresholdFilter`) meski sudah lolos gate onboard.

**4. Altitude yang disarankan:**
- **50-100 meter**: Coverage baik, resolusi tinggi
- Terlalu tinggi (> 150m): Detail berkurang
- Terlalu rendah (< 30m): Overlap terlalu sedikit

---

## GPS + Attitude Metadata pada Citra (EXIF)

Setiap gambar yang lolos IMU gate di `sender.py` disisipi metadata langsung ke file JPEG-nya (fungsi `embed_flight_metadata_exif()`), jadi **tidak perlu dikirim terpisah** — `stitcher.py` di ground tinggal baca ulang dari file-nya:

| Data | Lokasi EXIF | Format |
|---|---|---|
| Latitude / Longitude | `GPS GPSLatitude` / `GPS GPSLongitude` (GPS IFD standar) | DMS rational |
| Altitude | `GPS GPSAltitude` | Rational (meter) |
| Yaw / heading | `GPS GPSImgDirection` | Rational (derajat, 0-360, true north) |
| Roll & Pitch | `Image ImageDescription` (0th IFD) | JSON compact: `{"roll": 12.3, "pitch": -4.5, "yaw": 271.8}` |

EXIF tidak punya field baku untuk roll/pitch pesawat, jadi keduanya dititip di `ImageDescription` sebagai JSON — `stitcher.py` (`_extract_flight_metadata()`) sudah tau cara parse ini balik.

**Cek manual metadata di gambar hasil capture:**
```bash
python3 -c "
import exifread, json
with open('sessions/<session>/images/<file>.jpg', 'rb') as f:
    tags = exifread.process_file(f, details=False)
print('GPS:', tags.get('GPS GPSLatitude'), tags.get('GPS GPSLongitude'), tags.get('GPS GPSAltitude'))
print('Heading:', tags.get('GPS GPSImgDirection'))
print('Attitude:', json.loads(str(tags.get('Image ImageDescription'))))
"
```

---

## MAVLink UDP Fan-out (`fc_router.py`) — FC ke Banyak Program Sekaligus

**Masalah yang diselesaikan:** serial/COM port cuma bisa dibuka oleh SATU proses dalam satu waktu. Kalau mau jalanin `sender.py` dan `imu_monitor.py` (atau Mission Planner) BERSAMAAN ke FC fisik yang sama, program kedua akan gagal dengan `PermissionError: Access is denied`.

**Solusinya:** `fc_router.py` — baca serial FC SEKALI, forward semua data mentahnya ke beberapa port UDP sekaligus. Konsumen (`sender.py`, `imu_monitor.py`, dst) tinggal *listen* (`udpin`) di portnya masing-masing — pola koneksinya sama seperti `imu_simulator.py` (router = `udpout` aktif push, konsumen = `udpin` listen).

```bash
# Terminal 1 — router baca FC fisik, forward ke 2 listener sekaligus
python3 fc_router.py --port COM4 --baud 460800 \
    --out udp:127.0.0.1:14552 --out udp:127.0.0.1:14553

# Terminal 2 — sender.py listen di port pertama
python3 sender.py --fc-port udpin:0.0.0.0:14552 --host 192.168.1.100

# Terminal 3 — imu_monitor.py listen di port kedua, BERSAMAAN dengan sender.py
python3 imu_monitor.py --port udpin:0.0.0.0:14553 --no-send
```

Bisa juga forward sekalian ke Mission Planner/QGroundControl (biasanya listen di port `14550`):
```bash
python3 fc_router.py --port /dev/ttyAMA0 --baud 57600 \
    --out udp:127.0.0.1:14552 --out udp:127.0.0.1:14553 --out udp:192.168.1.50:14550
```

Port default (`14552`/`14553`) diatur di `config.py` (`FC_ROUTER_PORT_A`/`FC_ROUTER_PORT_B`), tinggal dipakai sebagai referensi biar konsisten di semua device — nggak wajib, port UDP mana pun bisa dipakai selama `--out` di router dan `--fc-port`/`--port` di konsumen match.

> **Kapan pakai ini vs koneksi serial langsung?** Kalau cuma jalanin SATU program (misal cuma `sender.py` doang) koneksi serial langsung (`--fc-port COM4`) masih lebih simpel, nggak perlu proses tambahan. `fc_router.py` baru kepake kalau butuh BEBERAPA program baca FC yang sama secara bersamaan.

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
