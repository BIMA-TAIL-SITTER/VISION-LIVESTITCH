# =============================================================================
# PROGRAM-NOVA | config.py
# Konfigurasi terpusat untuk semua program NOVA
# =============================================================================

# ── JARINGAN ─────────────────────────────────────────────────────────────────
GROUND_IP           = "192.168.1.100"   # IP laptop/GCS di ground (ganti sesuai jaringan)
UAV_IP              = "0.0.0.0"         # Bind semua interface di sisi UAV

# Port untuk stream gambar (UDP)
IMAGE_PORT          = 5600

# Port untuk data object detection / waypoint (UDP JSON)
OD_PORT             = 5601

# Port untuk telemetry / status IMU ke ground (UDP JSON)
TELEM_PORT          = 5602

# ── KAMERA ───────────────────────────────────────────────────────────────────
# Index kamera; DJI Osmo Action 5 Pro via USB biasanya /dev/video0 → index 0
CAMERA_INDEX        = 0
CAMERA_WIDTH        = 4096   # Resolusi horizontal (Osmo Action 5 Pro max 4K)
CAMERA_HEIGHT       = 3072   # Resolusi vertikal
CAMERA_FPS          = 5      # FPS capture (rendah untuk hemat bandwidth & latency)

# JPEG encode quality (0-100). 85 cukup baik & hemat bandwidth
JPEG_QUALITY        = 85

# Delay antar frame (detik).  1/CAMERA_FPS ≈ 0.2 s
FRAME_INTERVAL      = 0.5    # Ambil gambar tiap 0.5 detik (2 fps efektif)

# ── IMU / FLIGHT CONTROLLER ──────────────────────────────────────────────────
# Serial port FC ke Raspberry Pi
FC_SERIAL_PORT      = "/dev/ttyAMA0"   # UART GPIO; atau /dev/ttyUSB0 jika via USB
FC_BAUD_RATE        = 57600            # Sesuaikan dengan konfigurasi FC (Ardupilot default 57600)

# Threshold attitude: jika roll atau pitch melebihi nilai ini (derajat), streaming BERHENTI
ROLL_THRESHOLD_DEG  = 15.0   # > 15° dianggap sedang berbelok
PITCH_THRESHOLD_DEG = 20.0   # > 20° pitch tidak ideal untuk ortho
# Hysteresis: butuh N frame berturut-turut dalam kondisi aman sebelum streaming dimulai lagi
STABLE_FRAME_COUNT  = 5

# ── STITCHING ────────────────────────────────────────────────────────────────
# Jarak minimum (meter) antar gambar sebelum di-stitch (GPS-based threshold)
GPS_DISTANCE_THRESHOLD_M  = 3.0

# Jumlah gambar baru yang dikumpulkan sebelum auto-stitch dijalankan
STITCH_BATCH_SIZE         = 5

# Direktori sesi
SESSION_DIR               = "sessions"
DEFAULT_SESSION_ID        = "nova_session"

# ── OBJECT DETECTION SIMULATION ──────────────────────────────────────────────
OD_SEND_INTERVAL    = 10     # Kirim deteksi setiap N detik
OD_NUM_OBJECTS_MAX  = 3      # Maks objek per paket

# ── UDP BUFFER ───────────────────────────────────────────────────────────────
UDP_MAX_PACKET      = 65507  # Max UDP payload bytes
CHUNK_SIZE          = 60000  # Ukuran chunk untuk fragmentasi gambar besar
