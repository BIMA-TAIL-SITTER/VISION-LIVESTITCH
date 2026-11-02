# SENDER2 — live stitching UAV1 dan UAV2 terpisah

Implementasi revisi untuk dua UAV fixed-wing. **UAV1 dan UAV2 tidak digabung menjadi satu mosaic.** Masing-masing memiliki port UDP, antrean bounded, proses stitching, statistik, preview, dan direktori output sendiri. Belum ada integrasi OpenDroneMap.

Modul terpisah untuk **5 waypoint, radius 20 meter, satu fixed-wing, UDP + ACK** tersedia di [WAYPOINTS.md](WAYPOINTS.md). Modul ini tidak mengubah pipeline stitching dan belum mengendalikan copter.

Gunakan **`--flight FLIGHT-01`** untuk folder penerbangan berisi **`Image`** dan
**`Hasil`**, dengan output UAV1/UAV2 terpisah. Simulasi menerima `--dataset` untuk
UAV1 dan `--dataset2` untuk UAV2. Perintah lengkap dan struktur folder tersedia
di [FLIGHT_OUTPUT.md](FLIGHT_OUTPUT.md).

## Yang sudah diimplementasikan

- Schema metadata versi 2: mission/UAV/frame ID, UUID stream epoch, waktu capture dan clock domain, signed roll/pitch/yaw dalam derajat, usia telemetry, camera/calibration ID, dan tiga field altitude dengan datum berbeda.
- Arsip sebelum kirim: file replay disimpan byte-for-byte; frame kamera UVC disimpan sebagai PNG lossless dari frame hasil decode. Tidak ada GPS atau attitude palsu jika telemetry hilang.
- Preview JPEG dikirim melalui UDP datagram maksimum 1.200 byte. Reassembly memiliki validasi header/index, CRC32 seluruh frame, source binding, deduplication, batas memori dan TTL.
- Worker OS process per UAV melalui multiprocessing `spawn`, terpisah dari API. OpenCV thread dibatasi agar dua worker tidak memakai seluruh thread CPU masing-masing.
- SIFT + ratio test + RANSAC similarity; opsi rigid melakukan fit ulang termasuk translasi. Residual, jumlah/rasio/sebaran inlier, scale dan overlap diperiksa pada transformasi akhir.
- Kalibrasi pinhole OpenCV, undistortion dengan K yang diskalakan, dan validity mask terpisah dari warna piksel. Area hitam yang sah tetap masuk coverage.
- Cache keyframe hanya diperbarui setelah stitching sukses. Semua kandidat cache diuji geometrinya; registrasi gagal tidak memajukan reference.
- Recovery membuka segmen baru setelah kehilangan registrasi berulang, restart sender, perubahan geometri kamera, atau batas canvas. Segmen lama disimpan dan tidak disambung secara spekulatif.
- Preview rate-limited, error/status terlihat melalui API, file dipublikasikan atomik, shutdown menghentikan ingest lalu menyelesaikan antrean yang masih valid dan menyimpan hasil akhir.

## Batas penggunaan yang perlu dipahami

Output adalah **mosaik visual lokal dalam koordinat piksel**, bukan orthophoto/GeoTIFF dengan akurasi geografis. Tidak ada koreksi medan/DSM, bundle adjustment, atau loop closure global. Similarity merupakan pendekatan untuk area lokal relatif datar dan kamera mendekati nadir; relief, parallax, dan rangkaian panjang masih dapat menimbulkan drift. Blending tidak menjamin bebas ghosting.

Attitude badan UAV dipakai sebagai quality gate dengan asumsi mounting kamera tetap mendekati nadir. Program tidak mengubah course GPS menjadi yaw kamera dan tidak menerapkan matriks rotasi badan langsung ke piksel. Jika memakai gimbal/mounting miring, kontrak attitude kamera perlu ditambahkan dan divalidasi.

Timestamp UVC adalah **host_read_completed**, bukan timestamp exposure hardware. Telemetry dipilih dari buffer sampel terakhir sebelum waktu tersebut dalam domain host monotonic. Usia sampel tercatat, tetapi latency UART/FC/kamera tidak otomatis terkoreksi. Karena itu `capture_uncertainty_ms=null` dan `capture_clock=sender_host_utc_unverified`. Integrasi trigger kamera/TIMESYNC masih memerlukan hardware yang sesungguhnya. Program tidak mengklaim sudah memiliki sinkronisasi exposure presisi.

`altitude_relative_home_m` tidak dilabeli AGL. `altitude_agl_m` tetap null kecuali sumber pengukuran yang benar tersedia. GPS yang hilang tidak menghentikan registrasi visual; attitude yang hilang/stale ditolak oleh konfigurasi flight. Arsip onboard tetap dibuat sebelum frame preview diproses ground.

CRC32 adalah pemeriksaan korupsi, bukan autentikasi. Receiver ditujukan untuk link privat/tepercaya. API default hanya di `127.0.0.1` dan bersifat read-only. Tidak ada ACK/retransmit preview; kehilangan paket dideteksi lewat timeout sementara arsip onboard dipertahankan.

## Struktur

```text
SENDER2/
  __main__.py             CLI ground / sender / calibrate
  models.py               schema dan konfigurasi tervalidasi
  sender.py               capture/replay, archive, preview dan pacing
  telemetry.py            MAVLink buffer, signed units dan freshness
  protocol.py             fragmentasi/reassembly bounded
  geometry.py             kalibrasi, preprocess, rigid/similarity fit
  canvas.py               ROI blending dengan explicit coverage mask
  stitcher.py             state/cache/recovery per UAV
  worker.py               decode, engine, events dan publish
  runtime.py              UDP, admission, queue dan process lifecycle
  api.py                  REST API untuk base station
  calibrate.py            generator kalibrasi dari chessboard
  config.flight.json      dua UAV, calibration + attitude wajib
  config.replay.json      uji offline; missing telemetry diizinkan
  tools/replay_check.py   replay dua sender melalui UDP sungguhan
  tests/                 data, geometri, transport dan integrasi
```

## Instalasi

Jalankan semua perintah dari **folder induk SENDER2**, yaitu root VISION-LIVESTITCH pada checkout ini. Gunakan interpreter virtual environment yang sama untuk install dan menjalankan program.

Ground station:

```powershell
.\.venv\Scripts\python.exe -m pip install -r SENDER2/requirements.txt
```

Raspberry Pi (copy folder SENDER2, lalu jalankan dari folder induknya):

```bash
python3 -m venv .venv
source .venv/bin/activate
python -m pip install -r SENDER2/requirements-onboard.txt
```

Pymavlink hanya diimpor saat mode kamera dengan FC digunakan. Ground dan replay tidak membutuhkannya. Versi yang benar-benar teruji di workspace dicatat di [VALIDATION.md](VALIDATION.md); requirements menggunakan rentang versi, belum merupakan lockfile lintas-platform.

## Uji pertama tanpa kamera/FC

Terminal ground:

```powershell
.\.venv\Scripts\python.exe -m SENDER2 ground --config SENDER2/config.replay.json
```

Terminal sender UAV1:

```powershell
.\.venv\Scripts\python.exe -m SENDER2 sender --host 127.0.0.1 --port 5600 --mission replay01 --uav UAV1 --camera-id camera1 --replay-dir dataset/test_orthomosaics_42imgs --limit 12 --interval 0.8
```

Terminal sender UAV2:

```powershell
.\.venv\Scripts\python.exe -m SENDER2 sender --host 127.0.0.1 --port 5610 --mission replay01 --uav UAV2 --camera-id camera2 --replay-dir dataset/test_orthomosaics_42imgs --limit 12 --interval 0.8
```

Dataset yang sama di dua sender hanya untuk menguji isolasi. Untuk evaluasi operasi, gunakan dataset nyata UAV1 dan UAV2 masing-masing. Urutan replay mengikuti nama file; gunakan nama yang terurut berdasarkan capture. Replay ditandai `simulation=true` dan ditolak konfigurasi flight. Jika file memiliki sidecar SENDER2, telemetry diambil dari sidecar; file biasa memakai telemetry unknown, tanpa pose/GPS sintetis. Timestamp replay menunjukkan waktu pembacaan saat replay, bukan tanggal pemotretan asli; file asli beserta EXIF-nya tetap tersimpan.

Pantau [status dua UAV](http://127.0.0.1:8002/uavs), [preview UAV1](http://127.0.0.1:8002/uavs/UAV1/preview.jpg), [preview UAV2](http://127.0.0.1:8002/uavs/UAV2/preview.jpg), atau [API docs](http://127.0.0.1:8002/docs).

Tekan Ctrl+C pada ground untuk drain antrean dan menyimpan mosaic final. Jangan memakai `--reload` atau banyak Uvicorn worker karena setiap instance akan mencoba memiliki receiver/worker yang sama.

## Kalibrasi kamera sebelum mode flight

Ambil minimal 10 foto chessboard dengan sudut/posisi berbeda, pada kamera, resolusi, focal mode, crop, dan pengaturan stabilisasi yang akan dipakai saat capture. Angka cols/rows adalah **jumlah sudut internal**, bukan jumlah kotak. Nilai berikut hanya contoh pola 9 × 6 dengan kotak 25 mm:

```powershell
.\.venv\Scripts\python.exe -m SENDER2 calibrate --images calibration_photos/UAV1 --cols 9 --rows 6 --square-mm 25 --camera-id camera1 --calibration-id UAV1_fixedmode_v1 --output SENDER2/calibrations/UAV1.json
.\.venv\Scripts\python.exe -m SENDER2 calibrate --images calibration_photos/UAV2 --cols 9 --rows 6 --square-mm 25 --camera-id camera2 --calibration-id UAV2_fixedmode_v1 --output SENDER2/calibrations/UAV2.json
```

Tool menghasilkan K, koefisien distorsi, RMS dan error per foto. Periksa garis lurus serta residual pada foto validasi yang tidak dipakai untuk fit. RMS kecil sendiri belum membuktikan seluruh model benar. Tool ini menggunakan model **pinhole OpenCV**, bukan model fisheye; pilih mode kamera yang dapat dimodelkan dengan baik, atau tambahkan model fisheye teruji sebelum memakai lensa ekstrem.

Kalibrasi asli kamera pengguna tidak disertakan/dibuat-buat. `config.flight.json` sengaja gagal startup jika file kalibrasi belum ada. Resolusi capture harus sama dengan resolusi kalibrasi; resize preview boleh dilakukan tanpa crop/aspect ratio berubah. Mode kamera yang mengubah geometri secara dinamis memerlukan validasi tersendiri.

## Menjalankan dua pesawat

Pada ground, edit `mission_id`, port, camera_id, dan calibration_file dalam config flight. Path file kalibrasi dan output_dir dihitung relatif terhadap lokasi config.

```powershell
.\.venv\Scripts\python.exe -m SENDER2 ground --config SENDER2/config.flight.json
```

Pada Raspberry Pi UAV1 (ganti alamat ground dan port FC sesuai perangkat):

```bash
python -m SENDER2 sender --host 192.168.1.100 --port 5600 --mission flight01 --uav UAV1 --camera-id camera1 --calibration-id UAV1_fixedmode_v1 --fc /dev/ttyAMA0 --baud 57600 --width 1920 --height 1080 --interval 0.5 --max-mbps 8 --archive-dir /home/pi/sender2_archive
```

Pada Raspberry Pi UAV2:

```bash
python -m SENDER2 sender --host 192.168.1.100 --port 5610 --mission flight01 --uav UAV2 --camera-id camera2 --calibration-id UAV2_fixedmode_v1 --fc /dev/ttyAMA0 --baud 57600 --width 1920 --height 1080 --interval 0.5 --max-mbps 8 --archive-dir /home/pi/sender2_archive
```

Angka resolusi di atas harus diganti agar sesuai mode aktual kamera dan kalibrasinya. Setting OpenCV tidak menjamin perangkat menerima resolusi yang diminta; metadata memakai dimensi frame aktual dan ground memverifikasinya. Gate ground default: |roll|/|pitch| maksimal 30°, usia attitude maksimal 500 ms. Batas ini adalah parameter awal yang harus diuji untuk mounting dan kondisi misi kamu.

Capture terus menguras buffer kamera di thread tersendiri; pengirim mengambil frame terbaru setiap siklus. Interval adalah target minimum antar-frame terpilih, bukan jaminan 2 FPS. Penyimpanan PNG, encoding, CPU dan pacing jaringan dapat menurunkan laju aktual. `--max-mbps` berlaku **per UAV**, bukan total dua UAV. Sediakan ruang arsip yang cukup dan ukur bandwidth nyata.

## Integrasi dengan base station

| Endpoint | Kegunaan |
|---|---|
| `GET /health` | 200 saat worker ready/running; 503 saat startup, mati, atau shutdown |
| `GET /uavs` | Status kedua UAV, statistik jaringan/queue/worker, run directory |
| `GET /uavs/UAV1/status` | Progres dan error UAV1; UAV2 memakai ID sendiri |
| `GET /uavs/UAV1/preview.jpg` | Preview current segment UAV1, cache disabled |
| `GET /uavs/UAV1/segments` | Daftar segmen beserta alasan pergantian dan geometri pixel |
| `GET /uavs/UAV1/segments/0000/mosaic.png` | Raster final setelah segmen ditutup/shutdown |
| `GET /uavs/UAV1/segments/0000/coverage.png` | Coverage mask; hitam pada RGB bukan otomatis no-data |
| `GET /uavs/UAV1/segments/0000/segment.json` | Metadata origin, ukuran, epoch dan frame terakhir |

UI dapat polling status/preview sekitar 1 Hz. Tampilkan dua panel/layer terpisah: **UAV1 Live Mosaic** dan **UAV2 Live Mosaic**, dengan segment ID dan indikator belum georeferenced. Jangan menempatkan raster langsung ke peta geografis seolah memiliki koordinat valid; API ini memberi pixel-space preview. API tidak berisi frontend/modal base station lama dan belum memiliki dynamic attach/detach; daftar UAV/port ditentukan config saat startup. Koneksi UDP sendiri tidak memakai handshake connect/disconnect.

## Hasil dan arsip

```text
onboard_archive/<mission>/<UAV>/<stream_epoch>/
  000000000000.png atau .jpg   original yang dipertahankan
  000000000000.json           metadata + SHA-256
  manifest.jsonl

runs/<mission>/<run_id>/
  config.json
  received_previews/<mission>/<UAV>/<stream_epoch>/...
  UAV1/
    preview.jpg
    status.json
    events.jsonl
    network_events.jsonl      bila queue penuh
    segments/0000/
      preview.jpg
      segment.json
      mosaic.png              ketika ditutup
      coverage.png            ketika ditutup
  UAV2/                       state dan output tersendiri
  shutdown.json
```

Ground menyimpan **preview terkompresi**, bukan menggantikan arsip foto asli onboard. Tidak ada sinkronisasi arsip original otomatis pada versi ini. Run ID baru selalu dibuat pada ground startup agar hasil sebelumnya tidak tertimpa. State registrasi tidak dilanjutkan otomatis setelah ground restart.

## Overload, packet loss, reconnect dan shutdown

- Kapasitas queue default 4 frame encoded per UAV. Saat penuh, preview baru tidak dimasukkan queue; arsip preview yang sudah lengkap tetap disimpan, dan arsip onboard tetap ada.
- Frame yang berada terlalu lama di queue dibuang dengan status `stale_dropped`; threshold default 5 detik dari completion reassembly, termasuk waktu arsip. Metrik ini **bukan capture-to-display**, karena sinkronisasi clock lintas host belum dijamin.
- Chunk hilang menyebabkan frame expired; tidak pernah dikirim sebagai JPEG parsial ke engine. Cleanup berjalan meskipun tidak ada traffic. Batas buffer reassembly default 16 MB/UAV dan maksimal 16 frame pending.
- Frame ID lebih lama dari watermark dianggap terlambat dan dibuang. Tidak ada reorder window antarfoto; chunk dalam foto boleh datang acak. Kehilangan continuity ditangani cache/recovery, bukan memaksa posisi GPS.
- Restart sender menghasilkan UUID epoch baru dan frame ID mulai 0. Engine memulai segmen baru; paket epoch lama ditolak. Timestamp awal epoch harus meningkat; clock sender yang mundur ditolak secara terlihat melalui counter late_or_duplicate. Sinkronkan clock sebelum misi dan jangan mengubahnya di tengah sesi.
- Pada kegagalan registrasi keempat berturut-turut pada foto berfitur cukup, frame tersebut menjadi anchor segmen baru. Foto blur/attitude buruk tetap ditolak dan tidak dijadikan anchor baru.
- Batas canvas default 24 juta pixel per UAV (~384 MB dua buffer float32 saja, belum termasuk temporary/decoder). Mencapai batas membuka segmen baru. Ini belum tiled out-of-core renderer; target RAM harus ditentukan dari perangkat ground.
- Ctrl+C menghentikan UDP, drain queue yang belum stale, final publish lalu join proses. Deadline worker default 20 detik; bila terpaksa dihentikan, `shutdown_error.json` menyatakan final save tidak terjamin. Snapshot yang sebelumnya dipublikasikan atomik tetap dapat dibaca.
- Worker mati tidak di-restart diam-diam. `/health` menjadi gagal; receiver mengarsip preview dan melaporkan worker unavailable. Mulai run ground baru setelah penyebab diperiksa.

## Tes

Untuk simulasi seluruh DATASET 4 dengan dua UAV dan laporan otomatis, lihat [SIMULASI_DATASET4.md](SIMULASI_DATASET4.md). Jalankan `python -m SENDER2.tools.simulate_dataset --dataset "dataset/DATASET 4"` dari root repository.

```powershell
.\.venv\Scripts\python.exe -m pip install -r SENDER2/requirements-test.txt
.\.venv\Scripts\python.exe -m pytest SENDER2/tests -q -p no:cacheprovider
```

Suite meliputi signed telemetry/datum, arsip sebelum network error, ID tidak valid, transformasi known-truth dan outlier, rigid scale mismatch, explicit black-pixel coverage, kalibrasi/resolusi, quality gate, cache recovery, segmentation, chunk reorder/duplicate/loss/checksum, memory budget, epoch restart, dua proses UAV nyata via UDP localhost, endpoint API, dan shutdown final save. Artifacts uji disimpan di test_artifacts agar bisa ditinjau.

Replay dua sender pada dataset lokal melalui UDP sungguhan:

```powershell
.\.venv\Scripts\python.exe -m SENDER2.tools.replay_check --dataset dataset/test_orthomosaics_42imgs --count 12
```

Periksa `replay_report.json`, events, preview dan segment mask. Jalankan uji jaringan di lingkungan yang mengizinkan UDP loopback. Bila sandbox membuang datagram tanpa exception, tes unit protokol dapat lulus sementara tes localhost gagal; jangan menyebutnya lulus tanpa eksekusi jaringan yang benar.

## Referensi implementasi

- [OpenCV camera calibration](https://docs.opencv.org/4.13.0/dc/dbb/tutorial_py_calibration.html): intrinsik, distorsi dan pengambilan foto pola kalibrasi.
- [MAVLink common messages](https://mavlink.io/en/messages/common.html#GLOBAL_POSITION_INT): ATTITUDE dalam radian, GLOBAL_POSITION_INT altitude MSL dan relative_alt terhadap home; konversi ke satuan schema dilakukan sekali di telemetry.py.

Tahap berikut sebelum flight: kalibrasi dua kamera, instal dependency onboard, uji FC/camera di meja, replay dua stream selama durasi misi, lalu ukur registration success, queue age, error dan coverage pada data penerbangan yang benar-benar direkam. Integrasi ODM dilakukan terpisah setelah live pipeline ini dievaluasi di hardware.
