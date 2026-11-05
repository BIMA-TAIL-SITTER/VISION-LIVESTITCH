# Hasil simulasi DATASET 4 — 28 Oktober 2026

Program: [tools/simulate_dataset.py](tools/simulate_dataset.py). Panduan: [SIMULASI_DATASET4.md](SIMULASI_DATASET4.md).

## Input dan metode

- Dataset: `D:/KULIAH/BIMA - SWARM/VISION-LIVESTITCH/dataset/DATASET 4`.
- 275 JPEG berukuran 1920 × 1080; 273 gambar unik. Dua file `- Copy.jpg` identik byte-for-byte dilewati berdasarkan SHA-256. Dataset sumber tidak diubah.
- Semua 273 gambar unik dikirim ke UAV1 dan UAV2 secara paralel melalui UDP localhost. Dataset yang sama digunakan untuk kedua jalur; bukan dua penerbangan independen.
- Target interval pengiriman 0,8 detik; pacing 8 Mbps per UAV; JPEG preview lebar 1280, ukuran kerja lebar 960, similarity, antrean 4, usia antrean maksimum 5 detik. Gate geometri tidak dilonggarkan.
- Mode simulasi tanpa kalibrasi kamera atau telemetry; timestamp adalah waktu replay. Tidak mengikuti interval asli dari filename dan tidak menguji internet, DJI, Raspberry Pi, atau FC.
- Run lengkap: `runs/simulation/dataset_sim/20261028T051940Z_ce31602c`, durasi 228,98 detik (3 menit 49 detik), termasuk shutdown.

## Hasil run lengkap

| Metrik | UAV1 | UAV2 |
|---|---:|---:|
| Frame dikirim | 273 | 273 |
| Reassembly lengkap | 273 | 273 |
| Frame masuk antrean / diproses | 272 | 272 |
| Masuk mosaik, termasuk anchor | 157 | 157 |
| Gagal registrasi | 115 | 115 |
| Dibuang karena antrean penuh | 1 | 1 |
| Dibuang karena usia antrean | 0 | 0 |
| Error worker | 0 | 0 |
| Segmen | 28 | 28 |
| Segmen hanya satu frame | 10 | 10 |
| Median waktu worker | 459,8 ms | 455,2 ms |
| P95 waktu worker | 930,5 ms | 947,1 ms |
| Median usia antrean | 6,8 ms | 24,4 ms |
| P95 usia antrean | 2870,0 ms | 2900,1 ms |
| P95 waktu reassembly selesai hingga pemrosesan selesai | 3681,6 ms | 3732,1 ms |
| Status / exit code | stopped / 0 | stopped / 0 |

157 dari 273 input (57,5%) masuk hasil, termasuk 28 anchor segmen. Angka tersebut bukan keberhasilan menghubungkan semua gambar ke satu mosaik: hanya 129 frame ditambahkan ke segmen yang sudah memiliki anchor. Sebanyak 115 frame ditolak dengan alasan `no_valid_keyframe_match`. Semua pergantian dari segmen awal disebabkan `registration_lost`, bukan batas canvas.

Segmen 0003 memiliki frame terbanyak: 39 pada UAV1 dan 38 pada UAV2. Jumlah yang sedikit berbeda dapat terjadi karena posisi frame yang terbuang dan jalur registrasi masing-masing worker. Hasil kedua UAV tetap tersimpan terpisah. Waktu worker/antrean di atas bukan latency capture-to-display.

## Penilaian hasil

1. **Transport lokal dan lifecycle berjalan sampai selesai:** seluruh input diterima lengkap, kedua worker menyimpan hasil dan exit normal. Satu frame per UAV tidak masuk antrean karena kapasitas penuh. Ini tidak membuktikan keandalan radio/internet.
2. **Kontinuitas stitching belum memadai:** 28 segmen dan 10 segmen satu frame per UAV menunjukkan registrasi sering terputus. Run ini belum menghasilkan satu mosaik utuh untuk seluruh dataset pada setiap UAV.
3. **Kualitas visual masih bermasalah:** inspeksi preview UAV1 segmen 0003 menunjukkan ghosting/pelebaran tepi jalan dan bangunan, serta perubahan bentuk pada sambungan. Mosaik ini belum layak disebut orthophoto akurat.
4. **Kebutuhan diagnosis berikutnya:** evaluasi pasangan frame gagal beserta distribusi feature/residual, undistortion dengan kalibrasi aktual, dan kecocokan model similarity terhadap gerakan kamera/relief. Distorsi, parallax, feature berulang pada sawah, atau drift merupakan kemungkinan yang perlu diuji, bukan penyebab yang telah dibuktikan run ini.
5. **Evaluasi beban:** P95 worker sekitar 0,93–0,95 detik melebihi interval kirim 0,8 detik dan P95 antrean mendekati 2,9 detik. Pengujian interval lebih longgar dapat memisahkan masalah beban dari kegagalan geometri; jangan hanya melonggarkan batas registrasi agar jumlah frame diterima meningkat.

## File hasil yang bisa dibuka

- [Ringkasan otomatis seluruh segmen](runs/simulation/dataset_sim/20261028T051940Z_ce31602c/SUMMARY.md).
- [Laporan JSON lengkap](runs/simulation/dataset_sim/20261028T051940Z_ce31602c/simulation_report.json).
- [Preview UAV1 segmen 0003](runs/simulation/dataset_sim/20261028T051940Z_ce31602c/UAV1/segments/0003/preview.jpg).
- [Mosaik penuh UAV1 segmen 0003](runs/simulation/dataset_sim/20261028T051940Z_ce31602c/UAV1/segments/0003/mosaic.png).
- [Mosaik penuh UAV2 segmen 0003](runs/simulation/dataset_sim/20261028T051940Z_ce31602c/UAV2/segments/0003/mosaic.png).

Preview root menunjukkan segmen TERAKHIR, bukan gabungan seluruh segmen. Buka daftar segmen untuk melihat seluruh hasil. Folder `runs/` diabaikan Git; simpan/backup terpisah jika artefak gambar perlu dipindahkan.

## Kegagalan awal dan perbaikan yang dilakukan

Run pertama `20261028T051538Z_7a687f1f` berhenti setelah masing-masing sender mengirim 225 frame. Worker UAV2 gagal ketika `os.replace` mengganti `status.json` dengan `PermissionError / WinError 5`. Ini konsisten dengan konflik akses file Windows sementara, tetapi pemegang handle tidak diidentifikasi.

`storage.atomic_write()` sekarang mencoba ulang penggantian file maksimal 8 kali dengan jeda terbatas (total jeda hingga 0,71 detik). File lama yang lengkap tetap tersedia selama percobaan ulang; kegagalan permanen tetap dilemparkan, tidak disembunyikan. Run kedua diulang dari awal dan selesai tanpa error runtime. Tidak ada perubahan algoritme registrasi pada pengerjaan simulasi ini.

36 tes SENDER2 lulus setelah perbaikan, termasuk tes baru deduplikasi serta file terkunci sementara/permanen. Ada satu warning deprecation TestClient Starlette/httpx, bukan kegagalan tes. Hasil run gagal pertama tetap disimpan untuk audit.

## Menjalankan ulang

```powershell
.\.venv\Scripts\python.exe -m SENDER2.tools.simulate_dataset --dataset "dataset/DATASET 4"
```

Jalankan dari root VISION-LIVESTITCH. Setiap run menghasilkan folder baru, sehingga tidak menimpa hasil sebelumnya. Nilai timing/drop dapat berubah sesuai beban mesin.
