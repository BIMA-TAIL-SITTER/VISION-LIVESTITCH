# Debug Log: GPS Translation Sanity Check + Inlier Ratio Gate (Combiner.py)

> **Status**: In progress. Sanity check + inlier ratio + GPS-fallback chaining implemented and mostly working; spatial-spread check (§6 opsi #3 di `DRIFT_MISREGISTRATION.md`) belum diimplementasi, dan itu penyebab kink terbaru.
> **PENTING — klarifikasi klaim "kink hilang" (lihat §10)**: kink dari `ini_september` TERBUKTI gak muncul lagi di setelan `GPS_DISTANCE_THRESHOLD_M=3.0`+`inlier_ratio<0.6`, TAPI ini SUPPRESSED-BY-PARAMETER, bukan structurally fixed — begitu threshold diturunin ke 1.5, kink KAMBUH (beda frame pair, objek beda/rooftop bukan crop row, tapi MEKANISME SAMA: SIFT aliasing di tekstur repetitif). Root cause (§6/§7) BELUM tertutup tanpa spatial spread check.
> **Dataset**: real flight 2026-09-28, `uav_1`, 61 raw images (`20260928_164501_0000.jpg` s/d `20260928_195958_0060.jpg`). Sempat ke-delete gak sengaja waktu testing, dipulihin dari `~/.local/share/Trash/files/`.
> **Scope**: semua perubahan di `src/Combiner.py`, kelanjutan dari checklist `docs/DRIFT_MISREGISTRATION.md` §9.

---

## 1. Starting point: hasil "badut banget"

Abis implementasi awal GPS translation sanity check + RANSAC inlier ratio gate, full pipeline test (49 image pair) hasilnya HAMPIR SEMUA pair ketolak — mosaic akhir cuma 1 frame doang, gak ada stitching sama sekali.

Breakdown penyebab waktu itu:
- Inlier ratio (`<0.6`): sebagian kecil, wajar.
- GPS sanity check ("estimated translation too far"): MAYORITAS, gak wajar.

## 1.1 Kenapa inlier ratio bisa nolak walau tekstur "keliatan mirip"

Pertanyaan yang muncul di tengah investigasi: kalo dua frame farmland keliatan mirip-mirip aja teksturnya, kenapa inlier ratio-nya bisa jatoh?

Jawaban: inlier ratio ngukur apa MATCHED KEYPOINTS sepakat sama SATU transform geometris tunggal (RANSAC consensus) — bukan ngukur "seberapa mirip tekstur-nya secara visual". Di tekstur periodik (baris tanaman berulang), SIFT bisa nemuin match yang PERCAYA DIRI (lolos Lowe's ratio test dengan gampang) tapi nempel ke baris yang SALAH (baris ke-8 dikira baris ke-7, karena emang identik secara lokal). Titik "confidently wrong" ini valid secara visual, tapi GAK SEPAKAT sama titik yang bener soal transform yang seharusnya — RANSAC misahin mereka jadi inlier/outlier, dan kalo populasi dua kelompok itu deket-deket (misal 55:45), inlier ratio jatoh ke bawah threshold walau visual keseluruhan "mirip semua". Ini persis hipotesis `DRIFT_MISREGISTRATION.md §3` (SIFT descriptor aliasing).

Faktor tambahan yang bisa nurunin inlier ratio meski "keliatan mirip": terrain gak rata (gundukan/pohon/gedung kecil, ngerusak asumsi planar), motion blur/getaran (geser keypoint beberapa pixel), cahaya/bayangan berubah antar frame (gradient lokal berubah, basis SIFT descriptor ikut goyang).

## 2. Investigasi akar masalah (berlapis, beberapa hipotesis salah sebelum ketemu yang bener)

### 2.1 Hipotesis salah: padding-origin per-frame independen
Dicek: tiap image di `preprocess_images` di-`warpPerspectiveWithPadding` pake pose SENDIRI (bukan relatif ke frame sebelumnya), jadi origin kanvas tiap frame beda-beda. Dites empiris (skrip Python manual, bandingin translasi sebelum/sesudah dikoreksi offset padding) — **hipotesis ini SALAH**, offset-nya cuma 2-11px, gak signifikan, malah bikin errornya makin gede kalo "dikoreksi".

### 2.2 Bug real #1: origin-anchored vs center-anchored translation
`cv2.estimateAffinePartial2D` itu similarity transform (rotasi+scale+translasi). `A[0:2,2]` itu translasi TITIK (0,0) — bukan translasi KONTEN gambar. Kalo ada rotasi residual dikit aja (dan emang selalu ada), efeknya di-amplify parah karena origin (0,0) itu jauh dari tengah konten gambar (280-420px).

**Fix**: hitung translasi dari TITIK TENGAH gambar, bukan origin:
```python
h, w = image_shape[:2]
center = np.array([w/2, h/2, 1.0])
mapped_center = H_rel_3x3 @ center
mapped_center = mapped_center / mapped_center[2]  # perspective divide
translation_estimated_px = mapped_center[:2] - np.array([w/2, h/2])
```
Buktinya (data real): pair 0-1, `theta=-4.85°`, `t_origin=[16.74, 65.74]` vs `t_center=[5.73, 18.48]` — beda 3.5x.

### 2.3 Bug real #2: altitude MSL vs AGL
`meters_per_pixel` dihitung dari `FOV + altitude(EXIF) + image_width`, tapi altitude dari GPS EXIF itu **MSL** (mean sea level, absolut), bukan **AGL** (above ground level, relatif ke tanah) — padahal formula ground-coverage butuh AGL. `sender.py` (`PROGRAM_JUANG/PROGRAM-SENDER1/sender.py:190-195`) sebenarnya DIDESAIN nulis AGL (`self.alt_rel_m`, dari MAVLink `relative_alt`) ke EXIF, tapi dataset real yang dipake (GPS neo module, capture manual) kebaca MSL — gak lewat jalur konversi `sender.py` yang seharusnya.

Gak ada konstanta AGL tersimpan di manapun di repo (`config.py` manapun) buat dipake sebagai fallback fisik.

**Fix**: ganti dari physics-based (FOV+altitude) ke **kalibrasi empiris runtime** — kumpulin beberapa pair "sehat" (inlier ratio tinggi, GPS distance > 2m), hitung rasio `estimated_px / gps_m`, ambil median, invert jadi `meters_per_pixel`. Sekali kekalibrasi (butuh 5 sample), freeze buat sisa sesi (altitude di dataset ini nyaris konstan, jadi valid pake 1 scale buat semua frame).

### 2.4 Bug kalibrasi #1: inversi kebalik
```python
sample_scale = est_dist_px / gps_dist_m         # ini PIXEL PER METER
self._meters_per_pixel = np.median(samples)     # <- disimpen APA ADANYA, harusnya di-invert
```
Efeknya `tolerance_px ≈ 1px` — nolak semua lagi. **Fix**: `self._meters_per_pixel = 1.0 / np.median(samples)`.

### 2.5 Bug kalibrasi #2: perspective divide gak konsisten
Normalize (`mapped_center / mapped_center[2]`) cuma ada di blok kalibrasi, kelewat di `__check_gps_translation_sanity` sendiri. Ditambahin biar konsisten (relevan buat fallback jalur homography, affine gak kena masalah ini karena elemen ke-3 selalu 1).

### 2.6 Bug real #3: Y-axis sign convention
Abis dua fix di atas, run ulang (49 pair): 38 masih ketolak GPS sanity check. Dicek EXHAUSTIVE, **37 dari 38** nunjukin pola: estimated Y selalu NEGATIF, actual (GPS) Y selalu POSITIF — sign kebalik konsisten. **1 exception** (pair 47) beda arah total (bukan cuma sign, arah X-Y ketuker) — itu genuinely bad match, bukan bug.

Penyebab: GPS Y (dari lat, `flight_metadata.py:117`) itu ke UTARA (positif = utara), pixel Y (image row) itu ke BAWAH — dua konvensi beda, unrotation (`computeUnRotMatrix`) gak otomatis nyamain ini.

**Fix**: `translation_actual_m[1] = -translation_actual_m[1]` sebelum dikonversi ke pixel.

**Catatan penting**: flip ini KETEBAK BENER buat dataset ini karena heading pesawat konstan ~100° sepanjang flight. Belum tentu general buat heading lain (bisa jadi tergantung quadrant) — perlu tes ulang kalo ganti dataset/heading.

Hasil abis fix 2.2 + 2.6: dari 44 rejection sebelumnya, cuma 1 (pair 47) yang genuinely valid reject. Mosaic akhir BERSIH — kink lama (road dobel + patch pemukiman nyasar di `output/ini_september/output/finalResult.png`) HILANG.

---

## 3. Root cause kink lama, dikonfirmasi

Drone ngalamin **manuver bank/belok beneran** di ~frame 38-50 (roll naik bertahap dari ~2° ke puncak ~25-30°, dicek langsung dari EXIF). Bukan noise sensor 1 frame — sustained selama belasan frame. Asumsi corridor lurus (unrotation + affine planar) emang break di kondisi ini, jadi WAJAR direject.

`AttitudeThresholdFilter` (admission-level, threshold 30°) akhirnya nolak frame 59-60 di ujung flight (roll 30.1°), konsisten.

---

## 4. Diskusi arsitektur: "makin numpuk makin deket ground truth"?

User's argumen: makin banyak overlap, makin deket ground truth (logika bundle-adjustment).

**Koreksi**: project ini pake CHAINED homography (`H_global_current = H_global_prev @ H_rel_3x3`), BUKAN bundle adjustment. Di arsitektur chained, makin banyak frame di-chain = makin banyak KESEMPATAN ERROR NUMPUK (cumulative drift, udah didokumentasikan `TECHNICAL.md §10.1`), bukan makin akurat. Reject frame yang genuinely jelek itu nolongin, bukan ngerusak.

### 4.1 Tapi ketemu bug arsitektur REAL: chain-bridging gap
Waktu frame direject, `combine()` `return` tanpa update `H_global_prev`. Frame BERIKUTNYA tetep di-match lawan `image_list[index-1]` (posisi array, bukan "terakhir sukses"). Efeknya: transform "n relatif ke n-1" ke-chain dari posisi GLOBAL "terakhir sukses" (bisa beberapa frame sebelumnya), bukan dari posisi n-1 yang sebenarnya — jarak tempuh yang ke-skip dianggap NOL. Muncul sebagai lompatan/patah di mosaic.

**Opsi fix yang didiskusikan:**
1. Tetep advance `H_global_prev` pake `H_rel_3x3` dari SIFT walau direject — simpel, tapi resiko: kalo reject-nya emang karena transform beneran rusak (aliasing), tetep ke-chain, cuma pixelnya doang yang gak keliatan.
2. Bikin fallback translation-only DARI GPS (bukan dari SIFT yang gak dipercaya) buat advance posisi doang, blend tetep skip. **Dipilih** — lebih aman general, gak pernah nge-trust sinyal yang lagi diragukan.

**Estimasi biaya kode direvisi**: awalnya dikira opsi 1 jauh lebih simpel (gak perlu bikin fallback matrix baru). Tapi ketauan pas didalemin: DUA-duanya sama-sama butuh restrukturisasi yang sama (misahin "hitung canvas bounds + update posisi" dari "warp+blend+save", soalnya `H_global_prev` yang final itu `translation @ H_global_current`, `translation`-nya nempel ke proses canvas-bounds yang tadinya cuma jalan di jalur blend). Begitu restrukturisasi ini dianggap SAMA-SAMA WAJIB di dua opsi, delta kode opsi 2 di atas opsi 1 jadi tipis banget (cuma ~3 baris bikin matrix translasi dari GPS, datanya udah ada dari sanity check) — makin nguatin milih opsi 2.

**Kenapa gak hybrid (opsi 1 buat sebagian kasus, opsi 2 buat sisanya)**: dipertimbangkan (misal opsi 2 buat reject match-count/inlier-ratio rendah, opsi 1 buat reject yang SIFT-nya "yakin" tapi GPS-nya nolak) — TAPI ditolak. Alasan: nambah 3 kemungkinan sumber transform per frame bikin susah di-debug (gak jelas frame mana dapet transform dari sumber mana), sedangkan manfaat akurasinya tipis (abis Y-flip kefix, kasus "SIFT yakin tapi GPS beda" cuma 1 dari 38 kejadian — pair 47). Uniform pake opsi 2 buat semua jenis reject lebih gampang di-reason-in.

### 4.2 Implementasi opsi 2
- `__gps_fallback_transform(index)`: bikin matrix translation-only dari GPS delta (pake meters_per_pixel yang udah kekalibrasi + sign-flip convention yang sama). Return `None` kalo belum kalibrasi atau gak ada telemetry (zero pose) — never fabricate motion.
- `__advance_global_position(H_rel_3x3, image2_shape, will_paint=True)`: misahin "update posisi" dari "blend pixel". Dipanggil di SEMUA 5 titik reject (`descriptors None`, `matches<4`, `A_rel/H_rel None`, `inlier_ratio<0.6`, GPS sanity check fail) dan di jalur sukses.

**Bug pas implementasi**: sempet ada DUA definisi fungsi yang sama (`__gps_fallback_transform`, `__advance_global_position`) — kemungkinan collision antara edit aku sama edit user yang jalan bareng. Python diem-diem pake definisi TERAKHIR di file (yang gak ada komennya) — dead code silently. Ketauan pas review, diberesin (hapus duplikat).

**Bug lain pas testing pertama** ("malah tambah parah"): `__advance_global_position` SELALU ngerjain canvas-origin re-basing (`xMin,yMin` dari `__compute_canvas_bounds`, folded ke `H_global_prev`) — padahal re-basing ini CUMA valid barengan sama canvas beneran di-resize (`self.result_image` tumbuh). Di jalur reject, `result_image` gak berubah, tapi origin tetep "di-rebase" — `H_global_prev` jadi gak nyambung sama origin asli `result_image`. **Fix**: parameter `will_paint` — kalo `False`, `self.H_global_prev = H_global_current` langsung (tanpa re-base).

Abis fix ini: hasil membaik, gak separah sebelumnya.

---

## 5. Ghosting/blur di run terbaru (bukan kink, beda gejala)

Dibandingin `output/ini_september/output/intermediateResult_44.png` (baseline TANPA sanity check, TANPA inlier ratio) vs `sessions/uav_1/output/intermediateResult_43.png` (pipeline lengkap): area gudang/silo di versi baru ada ghosting/dobel tipis, junction jalan agak buram — baseline lama lebih tajam.

**Penjelasan (revisi)**: awalnya dikira bedanya dari ADMISSION filter (`GPSDistanceFilter`/`AttitudeThresholdFilter`) yang dikira gak ada di baseline lama. **Ini SALAH** — `ini_september` itu kondisinya SUDAH ADA flight metadata orchestrator juga (admission-level filtering sama kayak sekarang), dicek dari jumlah `intermediateResult_*.png`-nya (49 file, dari ~50 accepted images by 61 raw — angka ini cocok sama pola admission filter 3m, BUKAN sama dengan "semua 61 raw diproses tanpa filter").

Jadi beda density yang bikin ghosting itu BUKAN dari admission level (sama-sama ada di dua run) — itu murni dari **Combiner-level rejection** (inlier ratio + GPS sanity check) yang motong sebagian PASANGAN dari kumpulan frame admission yang SAMA. Baseline lama: SEMUA ~49 pasangan berhasil di-blend (gak ada gate apapun di Combiner). Pipeline baru: cuma 36-41 dari 49 pasangan yang lolos ke blend (sisanya di-fallback, gak dicat). Density overlap yang beneran nyampe ke `ROIfeatherBlender` itu yang beda, bukan jumlah raw frame yang di-admit.

**Lokasi konfigurasi**: `GPS_DISTANCE_THRESHOLD_M` sebagai NAMA konstanta cuma ada di config punya fork colleague (`PROGRAM_JUANG/.../config.py`), TIDAK dipake pipeline `service.py`/`src/` yang lagi ditest. Punya kita hardcoded default di `src/flight_metadata.py`:
```python
class GPSDistanceFilter:
    def __init__(self, threshold_m: float = 3.0):
```
dipanggil `service.py:62` tanpa override. User ngedit langsung jadi `GPSDistanceFilter(threshold_m=1.5)` buat eksperimen.

---

## 6. Threshold diturunin ke 1.5m — kink MUNCUL LAGI (masalah baru, bukan regresi dari fix sebelumnya)

Hasil: 38 frame sukses (naik dari 36), TAPI kink balik muncul — patch pemukiman miring/salah tempat di kanan-atas mosaic.

**Ditelusuri**: dua transisi mencurigakan, `matches_49_50.jpg` dan `matches_53_54.jpg` — DUA-DUANYA persis di zona manuver drone (roll 13-27°), dan DUA-DUANYA nunjukin pola match yang numpuk HAMPIR SEMUA di SATU AREA KECIL (atap rumah/pemukiman), cuma 1-2 match "nyasar" (garis panjang pojok-ke-pojok, kemungkinan false-positive) yang doang nyumbang spread.

**Root cause baru**: atap rumah di pemukiman itu JUGA tekstur repetitif (mirip-mirip antar rumah) — mekanisme SAMA kayak aliasing baris tanaman (`DRIFT_MISREGISTRATION.md §3`), cuma objeknya beda (buatan manusia, bukan crop row). Match yang numpuk di 1 klaster kecil BISA lolos inlier ratio (mereka "sepakat" satu sama lain secara lokal) dan BISA lolos GPS sanity check (translasi keseluruhan masih masuk akal), tapi TRANSFORM yang dihasilin gak reliable buat bagian frame yang JAUH dari klaster itu — muncul sebagai "melintir" di area lain.

Ini PERSIS opsi #3 di `DRIFT_MISREGISTRATION.md §6` ("Spatial spread check on matched keypoints") yang dari awal DITUNDA (cuma #1 GPS sanity + #2 inlier ratio yang diimplementasi). Nurunin `GPS_DISTANCE_THRESHOLD_M` ngebuka lebih banyak exposure ke frame zona manuver (deket pemukiman), jadi kemungkinan kena pola ini lebih tinggi.

**Belum diputuskan**: balikin threshold ke 3.0 (kurangin exposure, tapi ghosting balik) VS implementasi spatial spread check (nutup akar masalah, independen dari threshold).

---

## 7. Consideration: hapus inlier ratio gate?

Muncul pertanyaan: berhubung inlier ratio gate TERNYATA gak nangkep kasus rooftop-clustering (§6) — matched points yang numpuk tapi self-consistent bisa lolos inlier ratio dengan gampang — apa gate ini masih worth dipertahanin, atau dihapus aja (biar density overlap naik, ngurangin ghosting §5)?

**Pertimbangan HAPUS:**
- Gak nangkep failure mode rooftop-clustering (§6) — jadi manfaatnya buat kasus ITU nol.
- GPS sanity check udah ada sebagai gate independen — sebagian kasus low-inlier-ratio mungkin juga ke-tangkep di situ.
- Ngurangin 1 gate = lebih banyak frame lolos = density lebih tinggi = ghosting berkurang (§5).

**Pertimbangan PERTAHANIN:**
- Inlier ratio nangkep failure mode YANG BEDA dari GPS sanity check: match yang jumlahnya dikit/degenerate (deket floor `len(matches)>=4`) di mana FIT-nya sendiri underdetermined — bukan soal translasi salah, tapi soal TRANSFORM-nya gak cukup constrained buat dipercaya sama sekali (rotasi/scale bisa ngaco walau translasi kebetulan masih dalem toleransi GPS, karena GPS sanity check CUMA ngecek komponen translasi, gak ngecek rotasi/scale).
- Kasus reject inlier-ratio yang udah keliatan di run-run sebelumnya (pair 1, 6, 45, 46, 48, 49 — inlier ratio 0.43-0.59) itu match count-nya emang KECIL (4-25), bukan kasus "harusnya lolos tapi kena reject gak perlu".
- Gratis secara komputasi (mask udah dihasilin `cv2.estimateAffinePartial2D`/`findHomography`, tinggal itung rasio) — gak ada cost buat dipertahanin.
- Menghapusnya TIDAK menyelesaikan masalah §6 (rooftop clustering) — itu butuh spatial spread check (opsi #3), independen dari ada/gaknya inlier ratio gate.

**Rekomendasi**: JANGAN dihapus. Failure mode yang dia tangkep (match degenerate/dikit) beda dari yang GPS sanity check tangkep (translasi implausible) dan beda juga dari yang spatial spread check bakal tangkep (clustering). Ketiganya saling melengkapi, bukan redundan. Kalo tujuannya nambah density/ngurangin over-rejection, opsi yang lebih tepat: **turunin threshold-nya** (misal dari `0.6` ke `0.5`), bukan hapus total — biar tetep ada rem buat kasus match yang bener-bener degenerate.

---

## 7.1 Percobaan: inlier ratio diturunin ke 0.3 — hasilnya malah lebih berantakan

Hipotesis user: kalo inlier ratio diturunin (mendekati "gak ada gate"), hasil harusnya mendekati baseline lama (`ini_september`), karena baseline lama itu sendiri UDAH pake orchestrator flight_metadata di admission level (§5 revisi) — jadi kondisi admission-nya sebanding, tinggal Combiner-level gate-nya doang bedanya.

Dites: `inlier_ratio < 0.3` (reject) dan `inlier_ratio > 0.30` (calibration eligibility), turun dari `0.6`/`0.50`. Hasil: 41 frame sukses (naik dari 36), TAPI muncul 2 artefak baru:
1. Pemukiman kebelah jadi 2 patch, seam diagonal tajam.
2. Fragmen "melayang" gak nyambung (potongan kecil, isinya beda konten — jalan/kendaraan) di pojok kanvas.

**Root cause**: dua transisi fresh di run ini (`matches_48_49.jpg`, `matches_52_53.jpg`, timestamp dicek biar gak ketuker sama file stale run sebelumnya) nunjukin pola SAMA PERSIS kayak §6 — match numpuk di rooftop pemukiman, minim spread. Di threshold `0.6`, pasangan kayak gini ketolak, masuk fallback (gak dicat). Di threshold `0.3`, LOLOS inlier ratio, lanjut ke GPS sanity check — dan translasinya kebetulan masih masuk toleransi (GPS sanity check CUMA validasi translasi, BUKAN rotasi/scale) — jadi transform yang rotasi/scale-nya sebenernya ngaco itu ke-BLEND BENERAN, bukan di-skip. Itu penyebab langsung 2 artefak di atas.

**Kesimpulan**: hipotesis "turunin threshold = mendekati baseline lama" TERBUKTI SALAH. Baseline lama rapi bukan karena gate-nya longgar, tapi karena SEMUA pasangan (gak ada quality gate SAMA SEKALI) diproses — beda kualitatif, bukan cuma beda kuantitatif "seberapa longgar". Melonggarin gate yang ADA (inlier ratio) cuma ngebuka pintu buat transform yang UDAH kebukti rusak (rooftop clustering) buat kepasang beneran ke mosaic. Threshold dibalikin ke `0.6`.

**Temuan sampingan**: `sessions/uav_1/output/matches/` gak ke-clear antar run tempo hari (`matches_49_50.jpg`/`matches_53_54.jpg` sempat kepake sebagai "bukti" padahal file stale dari run sebelumnya, ketauan dari timestamp beda ~20 menit). Perlu clear `sessions/uav_1/output/matches/*` juga, bukan cuma `*.png` di root output, sebelum tiap run baru.

---

## 8. Status checklist `DRIFT_MISREGISTRATION.md` §9 (update)

- [x] Fix bare `return` di `Combiner.py:271` (lama)
- [x] GPS translation sanity check — **implemented + 3 bug fix (origin/center-anchor, MSL/AGL kalibrasi, Y-axis sign flip)**
- [x] RANSAC inlier ratio check — implemented
- [x] Fallback strategy pas transform direject — **opsi 2 (GPS-only translation, gak trust SIFT yang direject) dipilih & diimplementasi**, termasuk fix canvas re-basing (`will_paint`)
- [ ] **BARU**: Spatial spread check (opsi #3) — kebutuhan baru ketauan pas lower GPS_DISTANCE_THRESHOLD_M, matches numpuk di tekstur repetitif buatan manusia (rooftop pemukiman)
- [ ] Keyframe re-anchoring — belum disentuh, terpisah dari isu ini

## 9. Open items / belum diputuskan

1. Spatial spread check (opsi #3) — implementasi atau tidak, dan threshold-nya berapa (% coverage minimum dari bounding box matched points vs frame size).
2. `GPS_DISTANCE_THRESHOLD_M` final value — tradeoff ghosting (density rendah) vs exposure ke zona manuver (density tinggi).
3. Y-axis sign-flip fix (§2.6) — validasi ulang kalo ganti dataset dengan heading berbeda jauh, belum confirmed general.
4. Inlier ratio gate — **keputusan: dipertahankan** (§7), tapi threshold exact value (`0.6`) belum di-tuning ulang setelah semua fix di atas.

---

## 10. Status recap & rencana lanjutan (berhenti di sini, lanjut sesi berikutnya)

**State kode saat berhenti**: `src/Combiner.py` inlier ratio MASIH di `0.3`/`0.30` (eksperimen §7.1, terbukti lebih buruk) — **BELUM di-revert ke `0.6`/`0.50`**. `service.py:62` masih `GPSDistanceFilter(threshold_m=1.5)` (eksperimen §6, belum diputusin balikin ke `3.0` atau enggak). File `sessions/uav_1/output/finalResult.png` yang ada SEKARANG itu dari run threshold `0.3` (masih ada artefak pemukiman-kebelah + fragmen melayang) — BELUM di-rerun pake setelan bersih.

**Klarifikasi penting soal status "kink"** (jangan overclaim ke user lain / sesi depan): kink dari `ini_september` TERBUKTI gak muncul di setelan `3.0/0.6` (diverifikasi visual side-by-side), TAPI itu cuma SUPPRESSED-BY-PARAMETER — begitu `GPS_DISTANCE_THRESHOLD_M` diturunin ke `1.5`, kink KAMBUH lewat jalur berbeda (rooftop pemukiman, bukan crop row, tapi mekanisme SIFT-aliasing yang SAMA). Root cause (match numpuk di tekstur repetitif, gak ada spatial-spread check) BELUM tertutup.

**Rencana lanjutan, urutan disepakati:**
1. Revert `inlier_ratio` threshold: `0.3`→`0.6` (reject), `0.30`→`0.50` (calibration eligibility), di `src/Combiner.py`.
2. Implementasi **spatial spread check** (opsi #3, `DRIFT_MISREGISTRATION.md §6`) — prioritas utama sesi berikutnya, ini satu-satunya fix yang nutup akar masalah (aliasing di tekstur repetitif APAPUN objeknya), independen dari threshold manapun.
3. Abis spatial spread check tervalidasi, re-test `GPS_DISTANCE_THRESHOLD_M=1.5` lagi — cek apa density lebih tinggi (ngurangin ghosting §5) bisa dicapai TANPA kink balik, sekarang ada rem tambahan.
4. Keyframe re-anchoring — tetep di-park, terpisah, gak urgent.

**Gak ada kerjaan INTI yang perlu di-revert** — semua fix §2-4 (origin/center anchor, MSL/AGL kalibrasi empiris, Y-axis sign flip, chain-bridging GPS-fallback, canvas re-basing `will_paint`) valid & tervalidasi. Yang perlu direvert cuma eksperimen threshold hari ini (poin 1 di atas).
