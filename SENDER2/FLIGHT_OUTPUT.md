# Folder penerbangan: Image dan Hasil

Gunakan `--flight FLIGHT-01` untuk membuat folder bernama FLIGHT-01. Nama harus
1–32 huruf, angka, tanda minus, atau underscore. Nama folder yang sudah ada
ditolak supaya hasil penerbangan sebelumnya tidak tertimpa; gunakan FLIGHT-02
untuk run berikutnya.

## Simulasi dua dataset sekaligus

Jalankan dari folder utama VISION-LIVESTITCH di PowerShell:

```powershell
.\.venv\Scripts\python.exe -m SENDER2.tools.simulate_dataset --flight FLIGHT-02 --dataset "dataset/100GOPRO" --dataset2 "dataset/DATASET 4" --output SENDER2/runs --interval 1.2
```

`--dataset` adalah sumber UAV1, `--dataset2` adalah sumber UAV2. Ini penamaan
kanal simulasi, bukan bukti asal fisik foto dari wahana tersebut. Jika dataset2
tidak disebut, simulator memakai dataset pertama untuk kedua UAV seperti versi
sebelumnya. `--limit 10` boleh dipakai untuk uji singkat; tanpa opsi itu semua
foto unik dikirim. Deduplikasi berbasis isi file dan tidak menghapus sumber.

Struktur keluaran:

```text
SENDER2/runs/FLIGHT-02/
├── Image/
│   ├── UAV1/<stream_epoch>/
│   │   ├── 000000000000.jpg
│   │   ├── 000000000000.json
│   │   └── manifest.jsonl
│   └── UAV2/<stream_epoch>/...
├── Hasil/
│   ├── UAV1/
│   │   ├── preview.jpg
│   │   ├── status.json
│   │   ├── events.jsonl
│   │   └── segments/0000/
│   │       ├── mosaic.png
│   │       ├── preview.jpg
│   │       ├── coverage.png
│   │       └── segment.json
│   └── UAV2/...
├── onboard_archive/   # arsip asli pada simulasi
├── config.json
├── shutdown.json
├── simulation_report.json
└── SUMMARY.md
```

`Image` menyimpan JPEG preview yang berhasil diterima lengkap dan lolos validasi
identitas/urutan di ground, termasuk yang tidak masuk antrean stitching karena
antrean penuh. Gambar yang gagal diterima melalui UDP tidak dapat diarsipkan oleh
ground. File JSON mencatat nama sumber, identitas frame, dan metadata. Foto asli
tidak diganti oleh preview; salinan byte asli tetap di `onboard_archive` pada
simulasi, atau di Raspberry Pi saat penerbangan.

`Hasil/UAV*/preview.jpg` adalah preview **segmen aktif/terakhir**, bukan gabungan
seluruh segmen. Setiap `segments/*/mosaic.png` adalah hasil final suatu segmen;
file final ditulis ketika segmen ditutup atau program berhenti normal. Jika
registrasi gagal berulang, engine memulai segmen baru. Segmen tidak dipaksakan
menjadi satu peta dan hasil UAV1 tidak digabung dengan UAV2. Coverage mask
membedakan area berisi data dari area kosong. Ini mosaik live dalam koordinat
pixel, bukan orthophoto ODM atau GeoTIFF tergeoreferensi.

## Ground saat penerbangan

```powershell
.\.venv\Scripts\python.exe -m SENDER2 ground --config SENDER2/config.flight.json --flight FLIGHT-03 --output SENDER2/runs
```

Mode flight tetap memerlukan konfigurasi kalibrasi kamera dan telemetry yang
valid. Opsi folder tidak menonaktifkan pemeriksaan tersebut. API tetap mengambil
preview dari lokasi hasil yang baru.

Pada Raspberry Pi, gunakan `--flight FLIGHT-03` (alias `--mission FLIGHT-03`)
pada perintah sender, beserta host ground, port, UAV, kamera, dan FC yang sesuai.
Nama flight harus sama pada ground dan kedua sender. Contoh kerangka UAV1:

```bash
python -m SENDER2 sender --host ALAMAT_GROUND --port 5600 --flight FLIGHT-03 --uav UAV1 --camera-id camera1 --calibration-id ID_KALIBRASI --fc /dev/ttyAMA0 --baud 57600
```

Ganti placeholder dan parameter kamera/FC sesuai perangkat. Original archive
sender tetap dikontrol oleh `--archive-dir`. Berhenti normal dengan Ctrl+C agar
mosaik akhir ditulis. Mode tanpa `--flight` mempertahankan struktur run lama.

## Verifikasi perubahan

Suite SENDER2: 45 pengujian lulus. Integrasi UDP dua UAV diuji pada struktur lama
dan struktur Image/Hasil, termasuk preview API, packet loss, sender restart,
penutupan worker, dan penolakan nama flight yang sudah ada.
