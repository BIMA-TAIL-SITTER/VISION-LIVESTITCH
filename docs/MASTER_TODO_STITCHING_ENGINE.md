# Master To-Do: Stitching Engine (`src/Combiner.py`) — Merged Tracking

> Gabungan checklist dari `docs/DRIFT_MISREGISTRATION.md` (investigasi awal, kink farmland-aliasing) + `docs/CHAIN_BRIDGING_AND_THRESHOLD_CALIBRATION.md` (kelanjutan: chain-bridging bug, kalibrasi threshold, temuan GoPro) + `docs/TASK9_VERIFICATION_FINDINGS.md` (verifikasi full-pipeline yang PERTAMA KALI nemuin repro case kink `43↔44` dan nyatet to-do queueing/GCS). Dokumen ini CUMA tracking status — detail lengkap/narasi tetep ada di dokumen-dokumen sumbernya, plus `docs/GPS_SANITY_CHECK_DEBUG_LOG.md`.
> **Update terakhir**: 2026-09-29, abis sesi kalibrasi threshold + tes dataset GoPro.

---

## ✅ RESOLVED

- [x] **Bare `return` bug** (`DRIFT_MISREGISTRATION.md §7`) — `combine()` sekarang konsisten `return self.result_image` di semua early-exit.
- [x] **Konfirmasi hipotesis SIFT-aliasing di tekstur repetitif** (`DRIFT_MISREGISTRATION.md §8`) — kekonfirmasi lewat data real, frame pair `43↔44` (dataset lama) jadi repro case.
- [x] **GPS translation sanity check diimplementasi** (`DRIFT_MISREGISTRATION.md §6` item 1) — `__check_gps_translation_sanity`, dilengkapi:
  - [x] Bug origin-anchor vs center-anchor translation (amplifikasi rotasi) — fixed.
  - [x] Bug altitude MSL vs AGL (formula fisika FOV+altitude gak reliable) — diganti kalibrasi `meters_per_pixel` EMPIRIS runtime.
  - [x] Bug kalibrasi ke-invert (px/m disimpen jadi m/px) — fixed.
  - [x] Bug Y-axis sign flip (GPS utara vs pixel row ke-bawah) — fixed.
  - [x] **Toleransi (`TRANSLATION_TOLERANCE_M`) dikalibrasi DATA-DRIVEN** dari `4.5` (tebakan) → `10.0` (titik tengah gap kosong 7.79m-12.48m antara noise-corridor-biasa vs kink-zona-manuver). Lihat `CHAIN_BRIDGING_AND_THRESHOLD_CALIBRATION.md §6`.
- [x] **RANSAC inlier mask dipake, gak dibuang lagi** (`DRIFT_MISREGISTRATION.md §6` item 2) — `inlier_ratio < 0.6` sebagai gate accept.
  - [x] Bug dead-code: eligibility kalibrasi (`inlier_ratio > X`) awalnya ≤ gate accept, jadi gak pernah efektif nyaring apapun — fixed, sekarang eligibility (`0.7`) LEBIH KETAT dari gate accept (`0.6`), sengaja gitu (kalibrasi itu keputusan sekali-pakai-buat-selamanya, resikonya lebih tinggi kalo salah).
- [x] **Fallback strategy buat transform yang ditolak** (`DRIFT_MISREGISTRATION.md §6` item 6) — ini yang paling banyak makan waktu, 3 pendekatan dicoba SEMUA GAGAL dengan cara beda-beda sebelum ketemu yang jalan:
  - [x] ~~Selalu-bridge, translation-only~~ — ghosting nyebar ke seluruh mosaic (16/56 call, termasuk di area yang harusnya bersih). DIBUANG.
  - [x] ~~Selalu-bridge, yaw-aware (rotasi dari delta yaw)~~ — gak kebukti lebih baik dari translation-only. DIBUANG.
  - [x] ~~Skip total, match ke `last_accepted_index`~~ — collapse total, kejebak 1 index, gak pulih 46 frame sampe akhir flight. DIBUANG.
  - [x] **Bounded fallback (`__bridge_or_freeze`, `MAX_BRIDGE_GAP=3`)** — DIPILIH. Bridging cuma aktif kalo reject beruntun pendek, lewat itu freeze murni. Hasil crisp, divalidasi visual sebanding baseline `BestCombiner.py` tanpa gate.
- [x] **Spatial spread check** (`DRIFT_MISREGISTRATION.md §6` item 3, opsi tambahan yang sempet dieksplor) — diimplementasi (`__check_spatial_cross_validation`, core-vs-far residual check), TAPI kemudian **DIHAPUS** — superseded sama kombinasi bounded-bridge + threshold-retune yang hasilnya udah sebanding tanpa nambah kompleksitas.
- [x] **Chain-bridging gap bug** (ditemuin BARU pas sesi kalibrasi ini, gak ada di `DRIFT_MISREGISTRATION.md` awal — lihat bagian "BARU MUNCUL" di bawah buat konteks penemuannya) — resolved via bounded fallback di atas.
- [x] **Repro case konkret buat kink** (`TASK9_VERIFICATION_FINDINGS.md §5`) — frame pair `43↔44` (dataset lama, 2026-09-26) ke-identifikasi sebagai origin kink (cuma 4 match, 1 di antaranya garis diagonal implausible). Ini yang jadi validasi before/after buat semua fix di atas. Juga kekonfirmasi di §4: skew sistemik (bug degrees/radians) HILANG abis fix attitude-extraction, kink lokal masih ada (terpisah, yang akhirnya ditutup checklist di atas).

---

## 🟡 MASIH TERBUKA (dari checklist awal, belum disentuh)

- [ ] **Keyframe re-anchoring** (`DRIFT_MISREGISTRATION.md §6` item 5) — buat cumulative drift jangka panjang, SENGAJA dipisah dari bug misregistration single-frame ini. Belum digarap sama sekali.
- [ ] **Overlap density** (`GPS_DISTANCE_THRESHOLD_M`, sekarang `3.0`) — sekarang sanity check udah bener dikalibrasi, KEMUNGKINAN aman diturunin lagi (density lebih rapat, ghosting dari feather-blend jarang overlap berkurang) TANPA balik kena resiko kink zona manuver. **Belum dites.**
- [ ] **Bug kecil**: `dataMatrix[0] == [0,0]` by-construction (origin koordinat lokal) ke-detect SALAH sebagai "gak ada telemetry" di pengecekan fallback (`__simple_fallback_transform`). Dampak kecil (belum ada yang ketempel di titik itu), tapi logic-nya ambigu — perlu cek `has_telemetry` langsung, bukan infer dari posisi.
- [ ] **Auto-stitch queueing/batching** (`TASK9_VERIFICATION_FINDINGS.md §3, §6`) — dua masalah terkait, sama-sama nunjuk ke kebutuhan queue proper (bukan "baca apapun yang numpuk pas dibaca" kayak sekarang):
  - **(a) Gambar terakhir "yatim"** — `run_stitching()` baca `accepted_images` APAPUN yang udah numpuk pas dia mulai baca, bukan batch bersih kelipatan `auto_stitch_threshold` (5). Konsekuensi: gambar yang keterima PAS SETELAH 1 batch stitch mulai jalan bisa nyangkut gak ke-stitch sama sekali sampe ada trigger lagi (atau manual `/stitch`).
  - **(b) Karakteristik `combine()` yang N-1 dari N gambar** — `image_list[0]` jadi seed doang, gak pernah di-feature-match/di-validasi independen. Ini emang desain algoritma chaining sekarang (bukan bug queueing), tapi kalo desain queue baru motong-motong batch/re-anchor, tiap gambar pertama batch baru bakal kena karakteristik "gak pernah divalidasi" ini juga — (a) dan (b) perlu didesain bareng.
  - User punya ide, belum di-scope. Belum dimulai.

## 🚫 OUT OF SCOPE (diputusin sengaja gak dikerjain)

- [ ] Bundle adjustment / global optimization (`DRIFT_MISREGISTRATION.md §6` item 7) — kontradiksi filosofi desain real-time chained-homography project ini.
- [ ] Texture-aware feature masking (`DRIFT_MISREGISTRATION.md §6` item 8) — overkill, opsi #1+#2 udah cukup.

---

## 🆕 PROBLEM BARU YANG MUNCUL (gak ada di scope `DRIFT_MISREGISTRATION.md` awal)

Ini yang KETEMU pas ngerjain checklist di atas — bukan bug yang udah diprediksi dari awal, muncul organik dari testing.

### 1. Chain-bridging gap (udah RESOLVED, dicatet di sini biar keliatan asalnya)
Waktu frame ditolak, posisi mosaic (`H_global_prev`) gak ke-update — frame BERIKUTNYA jadi salah nge-chain (nganggep gerakan yang ke-skip itu nol). Muncul begitu inlier ratio + sanity check bikin reject rate naik dari ~12% (baseline lama) ke ~27-28% — bug LAMA (ada dari `BestCombiner.py` juga) yang jarang kepicu jadi SERING kepicu. 3 percobaan fix gagal sebelum landing di bounded-bridge (lihat checklist RESOLVED di atas). Full cerita: `CHAIN_BRIDGING_AND_THRESHOLD_CALIBRATION.md §2-4`.

### 2. Threshold gak portable lintas kamera — kemungkinan akibat distorsi lensa gak dikoreksi (🔴 OPEN, prioritas tinggi)
Dites ke dataset GoPro (`dataset/test_2`, lensa wide/fisheye) pake threshold yang udah dikalibrasi buat DJI — hasilnya: `inlier_ratio<0.6` nolak MAYORITAS frame, padahal baseline tanpa gate nunjukin registrasi mentahnya sebenernya FINE (cover area jauh lebih luas, gak ada kink parah). Hipotesis (belum kebukti): `inlier_ratio` ngukur kecocokan sama model SIMILARITY TRANSFORM (gak ada term distorsi lensa) — lensa wide/fisheye punya distorsi barrel asli yang bikin titik jauh dari tengah frame SECARA STRUKTURAL "gak cocok" sama model itu, walau correspondence-nya bener.

**To-do**: implementasi **lens undistortion PER CAMERA MODEL** (`cv2.undistort()`/`cv2.fisheye.undistortImage()`, kalibrasi SEKALI per model kamera — beda scope dari `meters_per_pixel` yang emang per-SESI). Ini jalan yang lebih bener ketimbang re-tuning threshold tiap ganti dataset/kamera — sesuai tujuan besar project: minim kalibrasi manual per deployment. Detail lengkap + tabel perbandingan scope kalibrasi: `CHAIN_BRIDGING_AND_THRESHOLD_CALIBRATION.md §9`.

**Belum dimulai.** Butuh: (a) cara deteksi kamera/profile per sesi (EXIF `Make`/`Model`), (b) koefisien distorsi per model kamera (kalibrasi checkerboard standar, sekali per kamera), (c) step undistortion ditambahin ke `__preprocess_images` sebelum downsample/unrotation yang udah ada.

---

## 🌐 To-Do Terkait, Beda Scope: Integrasi ke GCS (`TASK9_VERIFICATION_FINDINGS.md §6` item 3)

Ini scope-nya BUKAN `src/Combiner.py` lagi (deployment/integrasi level), tapi tetep dicatet di sini biar gak ke-lost. Konteks: user mau integrasi sementara/MVP stitching service ini ke base station GCS ("BIMA SWARM UGM"), 4 hal yang di-flag butuh perhatian:

1. **Resync urutan eksekusi pipeline** — router/service stitching di sisi GCS ditulis buat pipeline LAMA (`dataMatrix` Yaw/Pitch/Roll selalu nol, gak ada flight-metadata correction). Sekarang pipeline ini udah punya tahap attitude-extraction+filter beneran (`src/flight_metadata.py`) — bukan drop-in swap, perlu dicek ulang urutan/state di sisi GCS.
2. **Queueing/batching optimization makin urgent** kalo integrasi GCS beneran (tersambung sama to-do auto-stitch di atas). **Juga di-flag, belum di-RnD sama sekali: parallelism buat multi-UAV stitching** (beberapa UAV jalan stitching bersamaan) — belum diinvestigasi.
3. **UDP vs TCP buat socket image receiver perlu dibandingin ulang** — PoC kolega pake UDP (`PROGRAM_JUANG/.../sender.py`/`receiver.py`), punya kita (`receiver_socket.py`) pake TCP. Video pipeline GCS sendiri (BEDA dari stitching) udah pake UDP (port **5600**/**5601**, `docs/gcs_docs.md` §3.4) — preseden yang relevan buat dipertimbangin, bukan berarti harus ikut.
4. **Roadmap abis UDP settle**: kompresi video/HUD via WebRTC — `docs/gcs_docs.md` §3.4 udah nyatet video pipeline sekarang MJPEG-over-WebSocket, WebRTC itu roadmap item yang belum diimplementasi.

---

## 📌 Rencana User Selanjutnya (dinyatakan 2026-09-30)

Urutan komitmen user, bukan cuma saran aku — dicatat verbatim biar gak keubah pas sesi depan:

1. **Beresin `GPS_DISTANCE_THRESHOLD_M` (overlap density) dulu** — termasuk error-error lain yang kemungkinan nyusul muncul pas otak-atik ini (belum ketauan bentuknya, tapi user udah nyadar bakal ada).
2. **Abis itu, fokus PINDAH ke integrasi GCS**, spesifik 3 hal:
   - **Integrasi stitching service ke engine yang BARU** (`src/Combiner.py` versi sekarang — DAN nanti bakal ke-update LAGI, jadi integrasi ini bukan sekali-jadi, perlu desain yang tahan di-re-sync berkali-kali seiring engine terus berubah).
   - **Queue & batching optimization** (udah dicatat di atas — `TASK9_VERIFICATION_FINDINGS.md §3/§6`, orphaned trailing image + `combine()`'s N-1 characteristic).
   - **UDP vs TCP** — BUKAN cuma buat image receiver di sisi GCS, tapi JUGA sisi SENDER di UAV lapangan (dua sisi socket-nya, bukan cuma satu arah kayak yang kecatet sebelumnya).
3. **Lens undistortion (cross-camera)** dan **keyframe re-anchoring** — TETEP di-track (lihat di atas), tapi BUKAN fokus langsung berikutnya per rencana ini; kerjain kalo ada slot terpisah.

---

## Ringkasan Prioritas (referensi umum, di bawah rencana eksplisit di atas)

1. **Overlap density** (`GPS_DISTANCE_THRESHOLD_M`) — lagi dikerjain sekarang.
2. **Integrasi GCS** (engine sync + queueing/batching + UDP-vs-TCP dua arah) — fokus berikutnya per rencana user di atas.
3. **Lens undistortion** (cross-camera portability) — penting buat "plug-and-play" lintas kamera, tapi gak diprioritasin duluan dari integrasi GCS per rencana user.
4. Bug kecil `dataMatrix[0]` — low priority, fix kapan aja pas sempet.
5. Keyframe re-anchoring — jangka panjang, terpisah dari isu ini semua.
