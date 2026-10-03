# Master To-Do: Stitching Engine (`src/Combiner.py`) — Merged Tracking

> Gabungan checklist dari `docs/DRIFT_MISREGISTRATION.md` (investigasi awal, kink farmland-aliasing) + `docs/CHAIN_BRIDGING_AND_THRESHOLD_CALIBRATION.md` (kelanjutan: chain-bridging bug, kalibrasi threshold, temuan GoPro) + `docs/TASK9_VERIFICATION_FINDINGS.md` (verifikasi full-pipeline yang PERTAMA KALI nemuin repro case kink `43↔44` dan nyatet to-do queueing/GCS). Dokumen ini CUMA tracking status — detail lengkap/narasi tetep ada di dokumen-dokumen sumbernya, plus `docs/GPS_SANITY_CHECK_DEBUG_LOG.md`.
> **Update terakhir**: 2026-10-03, abis landing attitude drift gate (`__check_attitude_drift`) + overlap density `GPS_DISTANCE_THRESHOLD_M=2.0`, dan nemuin to-do baru soal collinear match/inlier_ratio.

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
- [x] **Overlap density (`GPS_DISTANCE_THRESHOLD_M`) diturunin ke `2.0`** — dites, sempet muncul "soft kink" baru (`intermediateResult_47/49/53.png`) karena 2 frame manuver yang BERSEBELAHAN bisa "konsisten" satu sama lain walau keduanya sama-sama udah drift jauh dari attitude normal (GPS sanity check cuma ngecek pasangan `index-1`↔`index`, gak nangkep drift bertahap). Root-cause ini confirmed via instrumentasi real (`matches_46_47.jpg` dibandingin `matches_43_44.jpg`, plus tabel roll/pitch delta per frame) — lihat `docs/explainer/COMBINER_FUNCTIONS_REFERENCE.md §3`.
- [x] **Attitude drift gate ditambahin** (`__check_attitude_drift`, user yang nulis) — bandingin roll/pitch frame sekarang vs `self.last_acc_index` (BUKAN `index-1`), independen dari SIFT/GPS-translation, dipanggil PALING AWAL di `combine()` sebelum SIFT jalan sama sekali. Nutup celah yang GPS sanity check gak nangkep (lihat poin di atas). Sempet ada bug (`self.last_acc_index = 0` ketimbang `= index` di jalur sukses — referensi gak pernah maju dari frame 0) — ketangkep & fixed pas code review sebelum dites. User udah tes ulang dataset `output/uav_1/images` (`GPSDistanceFilter(threshold_m=2.0)`, `ATTITUDE_DRIFT_TOLERANCE_DEG=7.5°`), HASIL DISUKAI user. Rentang `7°-9°` dicatet aman buat di-tuning ulang kalo perlu. **Trade-off yang perlu diinget**: di run tes ini, gate sukses nolak 47/49/53, TAPI abis frame `44` gak ada SATU PUN frame lolos semua gate sampe akhir data (56) — mosaic stall, sisanya cuma di-bridge/freeze. Belum ada mekanisme "pulih" begitu drone balik stabil pasca-manuver (lihat keyframe re-anchoring di bawah).

---

## 🟡 MASIH TERBUKA (dari checklist awal, belum disentuh)

- [ ] **Keyframe re-anchoring** (`DRIFT_MISREGISTRATION.md §6` item 5) — buat cumulative drift jangka panjang, SENGAJA dipisah dari bug misregistration single-frame ini. Makin relevan sekarang: attitude drift gate (RESOLVED di atas) nunjukin mosaic bisa "stall" abis manuver panjang (gak ada frame lolos sampe akhir data di satu run tes) karena gak ada cara buat drone yang udah balik stabil pasca-manuver "diterima lagi" tanpa nunggu balik deket attitude `last_acc_index` yang lama. Belum digarap sama sekali.
- [ ] **Bug kecil**: `dataMatrix[0] == [0,0]` by-construction (origin koordinat lokal) ke-detect SALAH sebagai "gak ada telemetry" di pengecekan fallback (`__simple_fallback_transform`). Dampak kecil (belum ada yang ketempel di titik itu), tapi logic-nya ambigu — perlu cek `has_telemetry` langsung, bukan infer dari posisi.
- [ ] **Match-distribution/conditioning check buat `inlier_ratio`** (🆕 ketemu pas investigasi kenapa frame awal `0↔1`, `1↔2` ketolak padahal visual-nya fine) — liat bagian "PROBLEM BARU" di bawah (#3) buat detail.
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

### 3. `inlier_ratio` bisa ke-gate FALSE NEGATIVE pas match-nya COLLINEAR/clustered (🆕 OPEN, medium priority)

Ketemu pas investigasi: dataset `output/uav_1/images`, pair `0↔1` dan `1↔2` (frame paling awal, abis takeoff) ketolak inlier ratio (`0.48`, `0.54` — di bawah gate `0.6`), padahal VISUAL kedua gambar itu "fine-fine aja" buat mata manusia.

**Root cause (udah diverifikasi liat match visualization asli, bukan dugaan)**: dibikin skrip debug buat nyimpen match inlier (ijo) vs outlier (merah) terpisah (`pair_1_2_inliers_green.jpg` / `pair_1_2_outliers_red.jpg`, scratchpad session). Hasilnya: KEDUA grup (yang dipertahanin RANSAC maupun yang dibuang) nunjukin korespondensi yang SECARA VISUAL BENER — gak ada garis nyilang/ngaco yang biasanya nandain mismatch. Masalahnya: scene-nya (farmland rata, low-texture) cuma ngasih SIFT keypoint yang KEBANYAKAN numpuk di SATU fitur kontras-tinggi doang (jalan/road, bentuknya garis tipis memanjang) — sisa frame (lahan kiri-kanan) nyaris gak ngasih keypoint. Set match yang HAMPIR COLLINEAR kayak gini itu secara matematis "poorly-conditioned" buat `estimateAffinePartial2D`/homography fitting — noise kecil (jalan gak benar-benar rata, parallax dikit, permukaan gak benar-benar planar) bikin RANSAC misahin match yang SAMA-SAMA BENER jadi inlier/outlier agak ARBITRARY, nurunin ratio walau registrasinya sendiri gak masalah.

**Beda dari masalah GoPro/lens-distortion (#2 di atas)**: itu soal MODEL transform-nya (similarity transform assumption rusak kena distorsi lensa). Ini soal GEOMETRI SEBARAN titik match-nya (collinear/clustered bikin RANSAC poorly-conditioned) — independen dari kamera/lensa apapun, bisa kejadian di kamera manapun kalo scene-nya kebetulan low-texture/didominasi 1 fitur linear.

**Dampak saat ini**: kecil/sementara — di dataset ini cuma mempengaruhi 2 frame paling awal (abis takeoff, sebelum masuk area bertekstur), terus "pulih sendiri" begitu scene lebih variatif. TAPI berpotensi muncul lagi di kondisi serupa (area urban dgn 1 jalan dominan, area gurun/lahan kosong, dll).

**To-do (belum di-scope detail, belum dimulai)**: pertimbangin nambah signal KEDUA selain `inlier_ratio` doang — misal spatial spread/variance dari titik match (udah ada preseden: `__check_spatial_cross_validation` yang DIHAPUS sebelumnya itu beda tujuan/udah superseded, tapi konsepnya bisa dipinjem ulang buat kasus ini secara spesifik), biar gate bisa BEDAIN "ratio rendah karena mismatch beneran" vs "ratio rendah karena match-nya collinear tapi sebenernya valid". Detail investigasi penuh: `docs/explainer/COMBINER_FUNCTIONS_REFERENCE.md` (liat diskusi sesi soal pair `1↔2`).

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

> **Status update (2026-10-03)**: Langkah 1 di atas (`GPS_DISTANCE_THRESHOLD_M` + attitude drift gate) SELESAI, hasil disukai user — liat checklist RESOLVED di atas. Langkah 2 (integrasi GCS) jadi fokus berikutnya, BELUM dimulai.

---

## Ringkasan Prioritas (referensi umum, di bawah rencana eksplisit di atas)

1. ~~**Overlap density** (`GPS_DISTANCE_THRESHOLD_M`)~~ — DONE, lihat RESOLVED.
2. **Integrasi GCS** (engine sync + queueing/batching + UDP-vs-TCP dua arah) — fokus SEKARANG per rencana user.
3. **Lens undistortion** (cross-camera portability) — penting buat "plug-and-play" lintas kamera, tapi gak diprioritasin duluan dari integrasi GCS per rencana user.
4. **Match-distribution/conditioning check buat `inlier_ratio`** — medium priority, ketemu organik pas tes overlap density, belum di-scope.
5. Bug kecil `dataMatrix[0]` — low priority, fix kapan aja pas sempet.
6. Keyframe re-anchoring — jangka panjang, tapi relevansinya naik abis temuan attitude-drift-gate "stall" (lihat RESOLVED + MASIH TERBUKA di atas).
