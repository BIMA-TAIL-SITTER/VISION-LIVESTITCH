# Validasi SENDER2 — 27 Oktober 2026

Implementasi live stitching dua UAV telah diuji secara otomatis dan melalui replay UDP lokal. Validasi ini belum merupakan uji terbang atau bukti akurasi pemetaan. Hasil UAV1 dan UAV2 disimpan terpisah.

## Lingkungan yang benar-benar digunakan

Windows, Python 3.13.7 x64, OpenCV 4.13.0, NumPy 2.5.0, Pydantic 2.13.4, FastAPI 0.138.2, Starlette 1.3.1, Pillow 12.2.0, Uvicorn 0.49.0, pytest 7.4.3, httpx 0.25.2. Interpreter: `.venv/Scripts/python.exe` pada root repository. `pymavlink` belum terpasang pada lingkungan pengujian; mode kamera/FC belum dijalankan.

## Hasil pengujian otomatis

```powershell
.\.venv\Scripts\python.exe -m pytest SENDER2/tests test/test_extract_gps.py test/test_import_data.py -q -p no:cacheprovider
```

**34 passed, 1 warning, 5.86 detik**: 32 kasus SENDER2 dan 2 tes regresi repository lama. Warning berasal dari deprecation integrasi TestClient Starlette/httpx pada versi terpasang; tidak menyebabkan tes gagal.

Pengujian mencakup:

- Metadata: signed attitude, konversi satuan dan datum altitude, unknown telemetry, ID/path tidak valid, nilai nonfinite, port duplikat.
- Arsip: file sumber byte-for-byte, frame kamera decoded disimpan PNG lossless, arsip terjadi sebelum error pengiriman, frame ID tidak dipakai ulang.
- Geometri: transformasi dengan ground truth sintetis, outlier, refit rigid beserta translasi dan residual, distribusi feature, kalibrasi/resolusi, coverage piksel hitam, batas canvas dan commit cache.
- Recovery: frame tidak berfitur tidak merusak reference, penolakan attitude hilang/stale/ekstrem, kegagalan registrasi berulang, dan segmen baru setelah epoch sender berubah.
- Transport: chunk acak/duplikat/hilang, checksum dan indeks invalid, source endpoint berbeda, isolasi UAV/epoch, batas memori, TTL, paket sesi lama setelah restart.
- Runtime: dua worker OS process melalui UDP localhost dengan isi gambar berbeda, API status/preview/segment, kapasitas queue, frame kedaluwarsa sebelum decode, decode error yang terlihat, serta penyimpanan final dan shutdown tanpa worker tertinggal.
- Konfigurasi flight menolak startup tanpa kalibrasi kamera.

Sandbox pada mesin ini membuang UDP loopback. Tes jaringan di atas dijalankan dengan izin eksekusi lokal yang mengizinkan UDP, setelah keterbatasan tersebut dikonfirmasi. Ini bukan hasil tes jaringan yang dimock.

## Replay dataset melalui dua sender

Dataset lokal: `dataset/test_orthomosaics_42imgs`, 12 file pertama `G0735851.JPG` sampai `G0735862.JPG`. Kedua sender menggunakan dataset yang sama untuk pengujian beban/isolation; ini bukan dua dataset penerbangan independen. Konfigurasi replay mengizinkan telemetry unknown dan tanpa kalibrasi. Resolusi kerja 960 pixel, similarity, cache 6, antrean 4 per UAV, target interval 0,8 detik, bandwidth maksimum pengujian 25 Mbps per UAV. Port localhost dipilih saat tes.

| Metrik | UAV1 | UAV2 |
|---|---:|---:|
| Frame dikirim / reassembly lengkap / diproses | 12 / 12 / 12 | 12 / 12 / 12 |
| Frame masuk mosaic, termasuk anchor segmen | 8 | 8 |
| Frame gagal registrasi dan tidak masuk mosaic | 4 | 4 |
| Queue drop / stale drop | 0 / 0 | 0 / 0 |
| Runtime error | 0 | 0 |
| Segmen output | 2 | 2 |
| Median waktu worker per frame | 326,8 ms | 303,9 ms |
| Persentil 95 waktu worker per frame | 561,8 ms | 485,0 ms |
| Status shutdown / exit code | stopped / 0 | stopped / 0 |

Waktu worker meliputi pemrosesan frame dan publish bila dijadwalkan; tidak mencakup seluruh capture, arsip onboard, transmisi, antrean, atau tampilan UI. Angka ini bukan latency capture-to-display atau jaminan FPS saat terbang. Tidak ada klaim peningkatan kecepatan terhadap program lama karena cakupan pengukuran berbeda.

Pada masing-masing UAV, segmen 0000 berisi anchor awal. Frame 1–3 gagal registrasi; frame 4 memulai segmen 0001 setelah ambang recovery tercapai. Frame 5 gagal registrasi, lalu frame 6–11 masuk segmen 0001. Segmen terakhir berukuran 1544 × 1511 pixel. Inspeksi visual masih menunjukkan sambungan/ghosting pada beberapa area. Penolakan dan recovery bekerja, tetapi hasil belum membentuk satu mosaic utuh seluruh 12 foto.

Laporan mesin tersimpan di [validation/replay_report.json](validation/replay_report.json). Artifact gambar dan events lokal ada di:

```text
SENDER2/test_artifacts/replay/audit_replay/20261027T031459Z_965ac03d/
  UAV1/segments/0000/ dan 0001/
  UAV2/segments/0000/ dan 0001/
  replay_report.json
  shutdown.json
```

`test_artifacts/` diabaikan Git; ringkasan ini dan salinan laporan mesin dapat dimasukkan version control. Untuk mengulang, jalankan `python -m SENDER2.tools.replay_check --dataset dataset/test_orthomosaics_42imgs --count 12` dari root repository.

## Yang masih perlu divalidasi pada perangkat nyata

1. Kalibrasi kedua kamera pada mode capture sebenarnya, mounting mendekati nadir, dan residual pada foto validasi terpisah.
2. Pembacaan MAVLink di dua pesawat, signed attitude, altitude datum, reconnect FC, serta timestamp/latency kamera dan telemetry. Timestamp UVC saat ini adalah waktu host selesai membaca frame; belum exposure hardware.
3. CPU, RAM, storage arsip PNG, bandwidth radio, packet loss/jitter, durasi misi panjang, reconnect sender, dan perubahan scene saat bank/turn.
4. Rasio registrasi berhasil, sebaran coverage, drift, serta ghosting pada dataset UAV1 dan UAV2 yang benar-benar berbeda. Jangan melonggarkan gate hanya agar semua frame diterima.

Keluaran tetap mosaic visual dalam koordinat pixel. Georeferencing, koreksi relief, sinkronisasi otomatis arsip original, integrasi UI base station lama, dan ODM belum diimplementasikan pada tahap ini. API dan dua output independen tersedia untuk integrasi live selanjutnya.
