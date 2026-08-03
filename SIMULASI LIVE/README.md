# Simulasi Live Stitching

Program simulasi **live stitching** yang mengirim gambar dari dataset melalui TCP socket dan langsung melakukan stitching secara incremental di sisi receiver.

## Arsitektur

```
Dataset (100GOPRO)                       Output
      │                                    │
      ▼                                    ▼
  sender.py ──TCP Socket──► receiver.py ──► output/stitch_0001.png
  (client)      port 5050    (server)       output/stitch_0002.png
                              │             output/stitch_0003.png
                              ▼             ...
                          stitcher.py       output/finalResult.png
                     (live stitching engine)
```

### Alur Stitching Incremental

```
Gambar 1 diterima → jadi mosaic awal
Gambar 2 diterima → stitch(mosaic, gambar_2) → stitch_0001.png
Gambar 3 diterima → stitch(mosaic, gambar_3) → stitch_0002.png
...dan seterusnya
```

## Cara Menjalankan

### Prasyarat

Pastikan dependencies sudah terinstall:
```bash
pip install opencv-python numpy
```

### Langkah-langkah

Buka **2 terminal** dan navigasi ke folder `SIMULASI LIVE`:

```bash
cd "d:\KULIAH\BIMA - SWARM\VISION-LIVESTITCH\SIMULASI LIVE"
```

**Terminal 1 — Jalankan Receiver terlebih dahulu:**
```bash
python receiver.py
```

**Terminal 2 — Jalankan Sender:**
```bash
python sender.py
```

Receiver akan mulai menerima gambar dan otomatis melakukan stitching. Hasil akan tersimpan di folder `output/`.

### Opsi Tambahan

**Sender:**
```bash
# Kirim hanya 10 gambar pertama (untuk testing cepat)
python sender.py --max-images 10

# Ubah delay antar gambar
python sender.py --delay 1.0

# Ganti dataset
python sender.py --dataset-dir "D:\path\ke\folder\gambar"

# Ganti port
python sender.py --port 6000
```

**Receiver:**
```bash
# Ganti port
python receiver.py --port 6000

# Ganti folder output
python receiver.py --output-dir hasil_stitch

# Ganti faktor downsample (lebih kecil = lebih detail tapi lebih lambat)
python receiver.py --downsample 3

# Simpan juga gambar yang diterima
python receiver.py --save-received
```

## Struktur File

```
SIMULASI LIVE/
├── sender.py      # Mengirim gambar dari dataset via socket
├── receiver.py    # Menerima gambar + langsung stitching
├── stitcher.py    # Modul stitching (self-contained)
├── README.md      # Dokumentasi ini
└── output/        # Hasil stitching (dibuat otomatis)
    ├── stitch_0001.png
    ├── stitch_0002.png
    ├── ...
    ├── finalResult.png
    └── matches/
        ├── matches_0001.jpg
        └── ...
```

## Contoh Output

Ketika dijalankan, terminal akan menampilkan:

**Terminal Receiver:**
```
============================================================
  SIMULASI LIVE STITCHING — RECEIVER
============================================================
  Listening   : 0.0.0.0:5050
  Output dir  : D:\...\SIMULASI LIVE\output
  Downsample  : 5x
============================================================

[RECEIVER] Menunggu koneksi di port 5050...

[RECEIVER] ✓ Terhubung dari ('127.0.0.1', 54321)

[RECEIVER] ── Gambar #1 diterima ──
  Ukuran: 2972.0 KB | Waktu terima: 0.012s
  Resolusi asli: 4000x3000
  ✓ Gambar pertama — mosaic diinisialisasi

[RECEIVER] ── Gambar #2 diterima ──
  Ukuran: 2975.1 KB | Waktu terima: 0.010s
  Resolusi asli: 4000x3000
  ⏱️  Feature Detection: 0.234s (1523 + 1487 keypoints)
  ⏱️  Feature Matching: 0.089s (42 good matches)
  ⏱️  Transform Estimation: 0.012s
  ⏱️  Warping: 0.045s
  ⏱️  Blending: 0.067s
  ✓ Stitch #1 selesai (0.65s)
```

**Terminal Sender:**
```
============================================================
  SIMULASI LIVE STITCHING — SENDER
============================================================
  Dataset    : D:\...\dataset\100GOPRO
  Jumlah     : 229 gambar
  Tujuan     : 127.0.0.1:5050
  Delay      : 0.5s antar gambar
============================================================

[SENDER] ✓ Terhubung ke receiver!

[SENDER] Mengirim gambar 1/229: G0030365.JPG ✓ 2972.0 KB (0.035s)
[SENDER] Mengirim gambar 2/229: G0030366.JPG ✓ 2975.1 KB (0.028s)
...
```

## Catatan

- Port default: **5050** (berbeda dari sistem utama yang menggunakan 5001, agar tidak konflik)
- Gambar di-downsample **5x** sebelum stitching untuk efisiensi
- Setiap hasil intermediate disimpan sehingga bisa dilihat progress stitching
- Tekan `Ctrl+C` di kedua terminal untuk menghentikan
