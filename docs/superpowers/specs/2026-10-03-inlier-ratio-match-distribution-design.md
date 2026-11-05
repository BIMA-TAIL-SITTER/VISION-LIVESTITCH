# Spec: Inlier Ratio False-Negative dari Collinear/Clustered Match Distribution

> **Status**: Belum ada implementasi diputusin. Dokumen ini CUMA nangkep diskusi + elaborasi opsi biar gak ke-lost, buat dilanjutin sesi depan. Cross-ref: `docs/MASTER_TODO_STITCHING_ENGINE.md` problem #3, `docs/explainer/COMBINER_FUNCTIONS_REFERENCE.md`.

---

## 1. Masalah

Dataset `output/uav_1/images`, pair `0↔1` dan `1↔2` (frame paling awal abis takeoff) ketolak gate `inlier_ratio < 0.6` (nilainya `0.48` dan `0.54`), padahal VISUAL kedua pair itu bener — diverifikasi langsung liat match visualization (`pair_1_2_inliers_green.jpg` / `pair_1_2_outliers_red.jpg`, scratchpad sesi ini), KEDUA grup (yang dipertahanin RANSAC maupun yang dibuang) nunjukin korespondensi yang BENER secara visual, gak ada garis nyilang/ngaco.

**Root cause**: scene (farmland rata, low-texture) cuma ngasih SIFT keypoint yang numpuk di SATU fitur kontras-tinggi — jalan (garis tipis memanjang). Set match yang HAMPIR COLLINEAR kayak gini "poorly-conditioned" buat `estimateAffinePartial2D`/homography fitting — noise kecil (jalan gak benar-benar rata, parallax dikit) bikin RANSAC misahin match yang SAMA-SAMA BENER jadi inlier/outlier agak ARBITRARY, nurunin ratio walau registrasinya sendiri gak masalah.

**Dampak sekarang**: kecil — cuma 2 frame paling awal di dataset ini, "pulih sendiri" begitu scene lebih variatif. Medium priority, bukan urgent.

---

## 2. Prior art: `__check_spatial_cross_validation` (udah dihapus)

Ada di `docs/dump/FUCKcombiner.py`. **PENTING**: fungsi ini solve masalah KEBALIKANNYA — dia NOLAK match yang numpuk di satu area kecil tapi gak generalize ke titik yang jauh (misal rooftop pattern berulang yang bikin SIFT match dengan percaya diri tapi SALAH). Docstring-nya malah eksplisit ngasih exception/escape-hatch ("no far points to test against -> accept") buat kasus "narrow-but-consistent, e.g. matches concentrated along a road" — PERSIS kasus kita sekarang.

Fungsi ini DIHAPUS karena superseded sama kombinasi bounded-bridge + threshold-retune (hasil udah sebanding tanpa kompleksitas tambahan), BUKAN karena konsepnya salah. Arsitektur sekitarnya (`ref_index = self.last_accepted_index` buat SIFT matching) itu bagian dari eksperimen yang GAGAL (catastrophic collapse) — jadi konsepnya bisa dipinjem lagi, TAPI implementasinya harus ditulis ulang di arsitektur sekarang (`index-1` chaining + bounded bridge), bukan di-copy-paste.

**Insight kunci dari fungsi lama ini**: dia punya cara BEDAIN "cluster yang bener" vs "cluster yang salah" — fit transform dari "core" cluster doang, terus cek apakah titik yang JAUH dari cluster itu (kalo ada) masih diprediksi bener sama transform itu. Kalo gak ada titik jauh buat dites → gak ada bukti, terima aja. Ini LEBIH presisi dibanding sekadar "ukur seberapa nyebar titiknya" (lihat §4).

---

## 3. Opsi yang dieksplor (buat gate behavior, BELUM diputusin)

### A — Turunin bar ratio pas spread sempit
Kalo matches numpuk/collinear (spread sempit), pake ratio threshold yang lebih longgar (misal `0.45` ketimbang `0.6`) khusus pair itu.
- (+) Perubahan minimal, `inlier_ratio` tetep sinyal utama.
- (-) Butuh threshold BARU ("apa itu 'sempit'") DI ATAS threshold ratio yang udah dilonggarin — dua angka baru, kontradiksi sama tujuan besar "minim kalibrasi manual". Masalah lebih besar: spread sempit BUKAN eksklusif buat kasus bagus — rooftop pattern berulang (kasus yang `__check_spatial_cross_validation` lama coba tangkep) JUGA bisa ngasih cluster rapat yang locally-agree tapi SALAH. Spread doang gak bisa bedain "jalan, match bener" vs "rooftop, kebetulan agree".

### B — Bypass `inlier_ratio` total, serahin ke GPS sanity check
Kalo spread sempit, skip gate ratio, biarin `__check_gps_translation_sanity` (udah jalan abis ratio, udah dipercaya sebagai "wasit/referee" di codebase ini) yang mutusin sendirian.
- (+) Gak ada angka relaksasi baru, pake sinyal independen yang udah dikalibrasi.
- (-) Chicken-and-egg: pair `0↔1`/`1↔2` itu PERSIS frame yang dipake buat kalibrasi `meters_per_pixel` itu sendiri — selama 5 sample kalibrasi belum kekumpul, `_meters_per_pixel is None`, GPS check gak bisa jalan sama sekali (cold-start gap yang emang udah ada di tempat lain). Juga gak solve disambiguasi: cluster rooftop yang salah translasinya bisa aja kebetulan GPS-plausible walau rotasi/scale-nya salah.

### C — Log doang, gak ubah behavior (✅ DIPILIH buat sekarang)
Hitung & print metrik spread di samping `inlier_ratio` buat SETIAP pair, gak ubah accept/reject behavior apapun. Ikutin metodologi yang udah dipake 2x sesi ini (`TRANSLATION_TOLERANCE_M`, `ATTITUDE_DRIFT_TOLERANCE_DEG`): kumpulin angka REAL dari banyak pair/flight dulu, baru cari gap kosong antara "spread dari reject yang BENERAN salah" vs "spread dari reject yang collinear-tapi-bener", baru putusin angka threshold-nya.
- (+) Resiko nol, data-driven (pola yang udah kebukti jalan di sesi ini), gak nebak angka.
- (-) Gak fix apa-apa hari ini (2 frame itu tetep ketolak). Diterima — dampaknya emang kecil/sementara.

**Keputusan sesi ini**: jalanin opsi C dulu. Opsi A dan B TETEP di-dokumentasiin di atas (bukan dihapus) biar scoping-nya udah ada pas mau lanjut ke situ nanti — tinggal liat hasil data dari C.

---

## 4. Metrik spread yang dipertimbangin buat opsi C (BELUM diputusin, deferred)

| Metrik | Cara hitung | (+) | (-) |
|---|---|---|---|
| **Bounding-box extent ratio** (rekomendasi awal) | Bounding box dari titik inlier, lebar/tinggi-nya dibagi lebar/tinggi gambar → angka `0-1` (`1`=nyebar sepanjang frame, deket-`0`=numpuk di satu titik) | Simpel, NORMALIZED (gak butuh kalibrasi, dan portable lintas kamera/resolusi — bonus, beda dari `inlier_ratio` yang udah kebukti camera-coupled) | Bounding box bisa "ketipu" kalo titiknya nyebar di 2 ujung doang (bimodal) tapi kosong di tengah — kesannya nyebar padahal sebenernya 2 cluster kecil |
| **Standard deviation posisi titik** | Std-dev koordinat x dan y dari titik inlier | Simpel juga | Satuan pixel mentah, gak dinormalize sama ukuran gambar/kamera — perlu konversi ekstra buat dibandingin lintas dataset |
| **PCA eigenvalue ratio** (collinearity measure) | Rasio eigenvalue dari PCA titik inlier — ngukur SEBERAPA GARIS LURUS sebarannya (bukan cuma seberapa jauh) | Paling presisi buat kasus spesifik "titik numpuk di satu garis" (kasus jalan ini persis) | Lebih susah dijelasin/dibaca sekilas di log dibanding 2 opsi di atas |

**Belum diputusin mana yang dipake.** User mau pertimbangin ini nanti — dokumen ini nyimpen elaborasi biar gak perlu diulang dari nol.

---

## 5. Yang BELUM di-scope sama sekali (buat diinget pas lanjut nanti)

- Metrik spread mana yang dipake buat logging (§4).
- Di mana persis logging-nya ditaro (tiap pair SEMUA, atau cuma yang ke-reject ratio? — buat gap-analysis valid, kemungkinan perlu DUA populasi: reject YANG BENERAN SALAH vs reject YANG COLLINEAR-TAPI-BENER vs accept biasa, jadi masuk akal log SEMUA pair bukan cuma yang ke-reject).
- Disambiguasi "cluster bagus" vs "cluster jelek" (§2-3) — kemungkinan butuh balikin konsep generalization-check dari `__check_spatial_cross_validation` lama, ditulis ulang di arsitektur sekarang, BUKAN cuma ukur spread doang. Ini mungkin jadi "fase 3" terpisah dari opsi A/B.
- Threshold angka buat opsi A (ratio relaksasi) atau kriteria "kapan bypass" buat opsi B — nunggu hasil data dari opsi C.

---

## 6. Next steps (urutan, belum dieksekusi)

1. (Nanti) Putusin metrik spread (§4).
2. (Nanti) Implementasi logging-only (opsi C) — tambah hitung+print metrik itu di `combine()`, gak ubah gate behavior apapun.
3. (Nanti) Jalanin di lebih banyak dataset/flight, kumpulin angka.
4. (Nanti) Gap-analysis kayak `TRANSLATION_TOLERANCE_M`/`ATTITUDE_DRIFT_TOLERANCE_DEG` — cari separasi antara populasi "reject salah" vs "reject collinear-tapi-bener".
5. (Nanti) Baru putusin opsi A, B, atau hybrid (dengan/tanpa disambiguasi generalization-check dari §2).
