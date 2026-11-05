# Simulasi DATASET 4

Jalankan dari root VISION-LIVESTITCH menggunakan virtual environment proyek:

```powershell
.\.venv\Scripts\python.exe -m SENDER2.tools.simulate_dataset --dataset "dataset/DATASET 4"
```

Program menjalankan dua sender simulasi, dua receiver UDP localhost, dan dua worker stitching. Tidak memerlukan kamera, Raspberry Pi, FC, atau aplikasi base station. API HTTP tidak dinyalakan; hasil dapat dibuka langsung dari file.

Dataset yang sama dikirim ke kedua UAV untuk menguji pemrosesan paralel dengan mosaik terpisah. Ini bukan dua rekaman penerbangan independen. Seluruh gambar unik dipakai secara default. Salinan dengan SHA-256 identik dilewati (file sumber tidak diubah); gunakan `--keep-duplicates` untuk memproses semuanya.

Parameter:

| Opsi | Default | Kegunaan |
|---|---|---|
| `--dataset` | `dataset/DATASET 4` | Folder foto, urutan nama file. |
| `--interval` | `0.8` | Target minimum detik antar pengiriman per UAV; tidak mengikuti timestamp filename. |
| `--max-mbps` | `8` | Pacing datagram per UAV, bukan emulasi jaringan internet. |
| `--limit` | `0` | Jumlah foto per UAV; 0 berarti seluruh gambar terpilih. |
| `--output` | `SENDER2/runs/simulation` | Folder hasil, setiap run memiliki direktori unik. |
| `--keep-duplicates` | tidak aktif | Mempertahankan input dengan isi identik. |

Foto dikirim sebagai preview JPEG maksimum lebar 1280; stitching menggunakan lebar kerja maksimum 960. Arsip file sumber disimpan byte-for-byte pada masing-masing arsip sender simulasi. Dataset asli tidak dimodifikasi. Telemetry tidak dibuat-buat; mode ini tidak mewajibkan attitude/kalibrasi.

Program menampilkan progres setiap sekitar 5 detik. Sesudah selesai, buka lokasi `SUMMARY` yang tercetak di terminal. `SENDER2/runs/simulation/latest_run.json` juga menunjukkan direktori run terakhir.

Keluaran:

- `SUMMARY.md`: ringkasan statistik dan tautan hasil setiap segmen.
- `simulation_report.json`: konfigurasi, daftar input/duplikat, statistik jaringan, worker, timing, coverage, dan shutdown.
- `UAV1/` dan `UAV2/`: preview, status, events, serta folder segmen independen.
- `segments/NNNN/mosaic.png`: mosaik penuh segmen yang ditutup.
- `segments/NNNN/coverage.png`: mask area berisi data.
- `onboard_archive/` dan `received_previews/`: arsip simulasi sumber dan preview yang diterima.

Frame masuk mosaik mencakup anchor pertama setiap segmen. Segmen baru dapat menandakan hilangnya registrasi atau batas canvas; jumlah frame diterima bukan ukuran akurasi. Periksa ghosting/drift secara visual. Mosaik memakai koordinat piksel, belum orthophoto/georeferenced. Waktu worker bukan latency capture-to-display. Uji ini tidak memverifikasi kamera DJI, hardware Raspberry Pi, MAVLink, atau internet.
