# Referensi Lengkap: Fungsi Baru/Diubah di `src/Combiner.py`

> **Tujuan dokumen**: buat dijelasin ke tim — semua fungsi yang ditambah/diubah dari versi awal (`docs/dump/BestCombiner.py`), APA fungsinya, KENAPA ada, dan DI MANA dipanggil.
> **Konteks lengkap/narasi debugging**: `docs/GPS_SANITY_CHECK_DEBUG_LOG.md` dan `docs/CHAIN_BRIDGING_AND_THRESHOLD_CALIBRATION.md`. Dokumen ini CUMA referensi fungsi, bukan cerita kronologis.

---

## 1. Gambaran Besar

Versi awal (`BestCombiner.py`) itu SIMPEL: tiap pasangan gambar di-cek pake 1 gate doang (`inlier_ratio < 0.6` → skip), sisanya LANGSUNG dipercaya dan ditempel ke mosaic. Resikonya: kalo SIFT ke-tipu tekstur repetitif (baris tanaman, atap rumah mirip-mirip), transform yang salah tetep numpuk gitu aja ke mosaic — nyebabin "kink" (gambar keplintir/numpuk salah tempat).

Yang ditambahin sekarang:
1. **GPS sanity check** — validasi tambahan: apa gerakan yang diukur SIFT itu MASUK AKAL dibanding data GPS asli.
2. **Kalibrasi meters-per-pixel otomatis** — biar sanity check itu bisa hitung skala pixel↔meter tanpa perlu tau spesifikasi kamera/ketinggian secara fisik (itu ternyata gak akurat, EXIF altitude itu MSL bukan AGL — liat `GPS_SANITY_CHECK_DEBUG_LOG.md` §2.3).
3. **Bounded fallback (`__bridge_or_freeze`)** — pas ada gambar yang KETOLAK (gate manapun), tetep jaga posisi mosaic biar gak "loncat" parah, TAPI dibatasin biar gak kebablasan.

---

## 2. State Baru di `__init__` (baris 41-51)

| Variabel | Nilai default | Fungsi |
|---|---|---|
| `self._mpp_calibration_samples` | `[]` | Nampung sample buat ngitung skala pixel↔meter (lihat §4). |
| `self._meters_per_pixel` | `None` | Skala pixel↔meter yang udah kekalibrasi. `None` = belum kalibrasi, sanity check & fallback OTOMATIS DI-SKIP sampe ini keisi. |
| `self._consecutive_rejections` | `0` | Counter — berapa kali REJECT BERUNTUN (reset ke 0 tiap ada yang sukses). |
| `self.MAX_BRIDGE_GAP` | `3` | Batas: cuma boleh "bridging" (lihat §5) kalo reject beruntun ≤ angka ini. Lewat itu, freeze total (gak diapa-apain). |

---

## 3. `__check_gps_translation_sanity(self, index, H_rel_3x3, image_shape)` — baris 282

**Fungsi**: Bandingin translasi (pergeseran) yang DIUKUR SIFT (dari `H_rel_3x3`, hasil matching gambar) VS translasi yang seharusnya SESUAI DATA GPS. Kalo bedanya kejauhan, transform itu DITOLAK (return `False`) — ini yang nyegah kink macam-macam kepasang ke mosaic.

**Cara kerja step-by-step**:
1. Toleransi ditentuin dalam METER (`TRANSLATION_TOLERANCE_M = 10.0`), dikonversi ke pixel pake skala kalibrasi.
2. Translasi SIFT diukur dari TITIK TENGAH gambar (bukan pojok/origin) — kalo diukur dari origin, rotasi kecil aja bisa bikin angkanya meleset jauh (efek "amplifikasi", udah dibuktiin empiris di awal investigasi).
3. Translasi GPS: selisih posisi `dataMatrix[index]` vs `dataMatrix[index-1]` (udah dikonversi ke meter lokal dari lat/lon). Sumbu Y di-BALIK (`translation_actual_m[1] = -...`) — soalnya GPS Y (utara) itu kebalik arah sama pixel row (ke bawah).
4. Kalo jarak antara dua translasi ini > toleransi → `False` (tolak), sekalian print warning.

**Kenapa toleransinya `10.0` meter (bukan angka sembarang)**: dihitung dari data real — dicek translation_error tiap pair yang ketolak, dipisahin mana yang "cuma noise korridor biasa" (maks ~7.79m) vs "beneran kink pas drone manuver" (minimal ~12.48m). `10.0` itu titik tengah gap kosong di antara dua grup itu. Detail lengkap ada di `CHAIN_BRIDGING_AND_THRESHOLD_CALIBRATION.md` §6.

**Dipanggil di**: `combine()`, abis kalibrasi mpp selesai (baris 422).

**GPS DIPOSISIKAN SEBAGAI REFERENSI** — bukan dikoreksi, bukan juga yang dikoreksi. GPS dipercaya sebagai "penggaris" (sinyal independen, gak ketipu tekstur repetitif kayak SIFT). SIFT itu yang DIUJI/DINILAI pake penggaris itu. Kalo SIFT nilainya gagal (bedanya kejauhan dari GPS):
- Angka SIFT-nya **GAK DIPERBAIKI** jadi angka GPS (gak ada "koreksi" numerik apapun).
- Yang kejadian: transform SIFT itu **DIBUANG TOTAL** (`return False` → `combine()` nge-skip frame ini, gak nempel ke mosaic sama sekali).
- Kalo mau ADA yang ngisi kekosongan itu, baru `__bridge_or_freeze` (§6) bikin gerakan PENGGANTI dari GPS doang (`__simple_fallback_transform`, §5) — TAPI itu FUNGSI LAIN, dan itu juga cuma buat "majuin posisi", BUKAN buat nge-blend pixel gambar.

Jadi ringkesnya: **GPS = wasit/pembanding. SIFT = yang diadili.** Kalo SIFT kalah, SIFT-nya dibuang (bukan dibetulin), bukan GPS yang diubah/disesuaikan ke SIFT.

**Asumsi yang perlu diinget**: check ini nganggep GPS itu "cukup benar" (GPS emang punya noise sendiri, biasanya beberapa meter, itu kenapa toleransinya gak 0 — ada slack `10.0m` buat nampung noise GPS). Kalo suatu saat GPS-nya sendiri yang error parah (misal glitch/jump sinyal), check ini malah bisa nolak SIFT yang SEBENERNYA BENER — itu resiko yang udah disadari, bukan dianggap gak mungkin terjadi.

**Contoh perhitungan (angka REAL dari sesi ini, `meters_per_pixel = 0.357971`, `tolerance_px = 10.0 / 0.357971 ≈ 27.94px`):**

*Kasus 1 — pair corridor biasa (27→28), HARUSNYA LOLOS:*
```
translation_estimated_px = [5.10, -27.55]   (dari SIFT, H_rel_3x3 @ titik tengah)
translation_actual_px    = [10.27, -40.32]  (dari GPS, udah di-convert & Y di-flip)

translation_error_px = norm([5.10-10.27, -27.55-(-40.32)])
                     = norm([-5.17, 12.77])
                     = 13.77px

13.77px < 27.94px (tolerance) → LOLOS ✅ (gak di-reject)
```

*Kasus 2 — pair zona manuver (46→47), HARUSNYA KETOLAK:*
```
translation_estimated_px = [2.56, -15.30]
translation_actual_px    = [30.81, 5.15]

translation_error_px = norm([2.56-30.81, -15.30-5.15])
                     = norm([-28.25, -20.45])
                     = 34.87px

34.87px > 27.94px (tolerance) → DITOLAK ❌ ("Estimated translation ... too far ...")
```
Beda jauh antara dua kasus ini (`13.77px` vs `34.87px`, gap kosong di tengahnya) itu PERSIS alasan kenapa toleransi `10.0m` dipilih — lihat §6 di `CHAIN_BRIDGING_AND_THRESHOLD_CALIBRATION.md` buat liat SEMUA pair yang dipake nentuin angka ini.

---

## 4. Blok Kalibrasi `meters_per_pixel` (inline di `combine()`, baris 402-417)

**Fungsi**: Nentuin skala "1 pixel di gambar = berapa meter di tanah" TANPA perlu tau spek kamera/altitude fisik (yang ternyata gak reliable — EXIF altitude itu MSL/ketinggian laut, bukan AGL/ketinggian dari tanah).

**Cara kerja**:
1. Cuma jalan kalo `self._meters_per_pixel is None` (belum kalibrasi).
2. Syarat pair boleh jadi "sample" kalibrasi: `inlier_ratio > 0.70` (LEBIH KETAT dari gate accept `0.6` — sengaja, soalnya kalibrasi ini nentuin skala buat SELURUH sisa sesi, jadi harus dari match yang BENERAN bagus) DAN jarak GPS `> 2.0m` (biar gak dibagi angka kecil/gak stabil).
3. Ngitung rasio `pixel_bergerak / meter_bergerak` (pake translasi center-anchor, SAMA formula kayak sanity check).
4. Begitu udah kekumpul 5 sample, ambil MEDIAN-nya (bukan rata-rata — biar tahan kalo ada 1-2 sample nyasar), di-INVERT (`1/median`) jadi `meters_per_pixel`, terus DI-FREEZE (gak berubah lagi) buat sisa sesi.

**PENTING — bug yang pernah kejadian**: syarat `inlier_ratio > 0.70` ini HARUS lebih tinggi dari gate accept (`0.6`). Kalo disamain atau lebih rendah, syarat ini jadi PERCUMA (dead code) — soalnya kode ini CUMA kejalanin buat frame yang UDAH LOLOS gate accept, jadi `inlier_ratio` udah pasti di atas `0.6` duluan.

**Contoh perhitungan (ilustrasi — pola angka kayak gini yang ngasilin `0.357971` yang kepake di sesi ini):**
```
5 sample scale yang kekumpul (px/meter), tiap kali pair lolos syarat ratio>0.7 & jarak>2m:
  sample 1: 2.81 px/m
  sample 2: 2.75 px/m
  sample 3: 2.90 px/m
  sample 4: 2.79 px/m
  sample 5: 2.74 px/m   <- ada 1 yang agak nyasar dikit, biasa

median([2.81, 2.75, 2.90, 2.79, 2.74]) = 2.79 px/m   (median tahan outlier, bukan rata-rata)

meters_per_pixel = 1 / 2.79 ≈ 0.3584 m/px
```
Abis ini, `self._meters_per_pixel` FREEZE di angka itu — sample ke-6 dst GAK NGARUH lagi, gak pernah dihitung ulang sepanjang sesi.

---

## 5. `__simple_fallback_transform(self, index)` — baris 207

**Fungsi**: Bikin transform TRANSLATION-ONLY (gerak doang, gak ada rotasi) LANGSUNG dari data GPS — dipake SEBAGAI GANTI hasil SIFT, KHUSUS pas SIFT-nya lagi ditolak/gak dipercaya.

**Kenapa translation-only (gak pake rotasi)**: sempet dicoba versi yang ngitung rotasi juga (dari selisih `yaw`), TAPI dites gak kebukti lebih bagus (lihat `CHAIN_BRIDGING_AND_THRESHOLD_CALIBRATION.md` §3.2) — jadi dibalikin ke versi simpel.

**Pengaman di dalamnya**:
- Kalo `meters_per_pixel` belum kekalibrasi → return `None` (gak bisa ngitung apa-apa).
- Kalo posisi GPS `prev_pos`/`curr_pos` itu PERSIS `[0,0]` → dianggap "gak ada data telemetry", return `None` (JANGAN NGARANG gerakan dari data kosong).
  - ⚠️ **Catatan**: frame index 0 itu SELALU `[0,0]` by design (dia jadi titik origin koordinat lokal), BUKAN karena gak ada telemetry — jadi buat pair pertama (0→1), pengecekan ini bisa salah tangkep. Dampaknya kecil (belum ada apa-apa yang ketempel di mosaic di titik itu), tapi perlu diinget.

**Dipanggil dari**: `__bridge_or_freeze` doang (§6).

**Contoh perhitungan (ilustrasi, pake `meters_per_pixel` real `0.357971`):**
```
prev_pos (GPS, meter lokal) = [0.00, 0.00]
curr_pos (GPS, meter lokal) = [1.20, 4.50]

delta_pos_m = curr_pos - prev_pos = [1.20, 4.50]

translation_px = delta_pos_m / meters_per_pixel
              = [1.20/0.357971, 4.50/0.357971]
              = [3.35, 12.57]

translation_px[1] = -translation_px[1]   # flip Y
              = [3.35, -12.57]

H_fallback = [[1, 0,  3.35],
              [0, 1, -12.57],
              [0, 0,  1]]
```
Matrix ini LANGSUNG dipake buat majuin `H_global_prev` (lewat `__advance_global_position`, `will_paint=False`) — gak ada rotasi/scale sama sekali, murni geser.

---

## 6. `__bridge_or_freeze(self, index, image2_shape)` — baris 238

**Fungsi**: Ini "otak" dari keputusan APA YANG TERJADI PAS SEBUAH FRAME DITOLAK. Dipanggil di SEMUA 5 titik reject di `combine()`.

**Logic**:
```
counter reject += 1
KALO counter <= MAX_BRIDGE_GAP (3):
    coba bikin fallback transform dari GPS (§5)
    KALO berhasil: majuin posisi mosaic pake itu (TANPA nempel pixel)
KALO counter udah lebih dari 3:
    diem aja, gak ngapa-ngapain (freeze)
```

**Kenapa dibatasin (gak selalu "bridging")**: udah dicoba SELALU bridging (gak dibatasin) — hasilnya ghosting/blur nyebar ke MANA-MANA di mosaic, bahkan di area yang harusnya bersih. Ternyata SATU KALI PUN pake approksimasi translation-only ini udah nyisipin error kecil yang keliatan pas di-blend. Jadi dibatasin biar gak KEBABLASAN dipake terus-terusan, TAPI tetep ada buat nutup celah "posisi ke-freeze pas ada reject" (§7).

**Kenapa gak "SELALU freeze" aja (gak usah ada bridging sama sekali)**: kalo freeze total, tiap kali ada reject, frame BERIKUTNYA bakal "loncat" posisi (ke-chain dari titik 2-frame-sebelumnya, bukan yang seharusnya) — muncul sebagai patahan/staircase di mosaic. Bridging (dibatasin) itu jalan tengah.

---

## 7. `__advance_global_position(self, H_rel_3x3, image2_shape, will_paint=True)` — baris 252

**Fungsi**: Nge-CHAIN posisi (`H_global_prev`) — ini fungsi INTI yang nentuin "sekarang mosaic udah nyampe posisi mana". Dipake DI DUA JALUR:
- **Jalur sukses** (`will_paint=True`, default): abis semua gate lolos, majuin posisi DAN siapin buat nge-blend pixel beneran (`xMin/yMin/xMax/yMax` buat ukuran kanvas).
- **Jalur reject-tapi-di-bridging** (`will_paint=False`, dipanggil dari `__bridge_or_freeze`): majuin posisi DOANG, TANPA nempelin pixel apapun ke mosaic.

**Detail teknis penting**: kalo `will_paint=False`, fungsi ini SENGAJA GAK ngerjain "re-base origin kanvas" (`xMin`/`yMin` dari `__compute_canvas_bounds`) — soalnya re-basing itu CUMA valid bareng sama canvas yang BENERAN di-resize. Kalo dipaksain re-base padahal kanvas gak berubah, `H_global_prev` bakal "nyasar" dari origin asli `self.result_image` — pernah kejadian bug ini pas awal development, makanya ada pemisahan `will_paint`.

**Perbandingan sama `BestCombiner.py`**: formula chaining-nya (`H_global_prev @ H_rel_3x3`, normalize, dst) itu **IDENTIK** sama versi lama. Bedanya CUMA fungsi ini dipanggil lebih sering (2 kondisi: sukses ATAU bridging), sedangkan `BestCombiner.py` cuma manggil versi setara ini 1x per frame (pas sukses doang, gak punya konsep bridging).

---

## 8. Alur Lengkap `combine(index)` — Ringkasan Gate

```
1. Deteksi fitur (SIFT) di image[index-1] vs image[index]
   └─ kalo GAK ADA fitur sama sekali → __bridge_or_freeze() → skip

2. Matching fitur (BFMatcher + ratio test)
   └─ kalo match < 4 → __bridge_or_freeze() → skip

3. Estimasi transform (affine/homography)
   └─ kalo GAGAL total → __bridge_or_freeze() → skip

4. Cek inlier ratio (dari RANSAC)
   └─ kalo < 0.6 → __bridge_or_freeze() → skip

5. [kalo meters_per_pixel belum kekalibrasi]
   → coba kalibrasi (butuh inlier_ratio > 0.7 & jarak GPS > 2m, 5 sample)

6. GPS sanity check (translasi SIFT vs translasi GPS)
   └─ kalo beda > 10m → __bridge_or_freeze() → skip

7. SEMUA LOLOS → reset counter reject, __advance_global_position(will_paint=True)
   → warp + blend pixel beneran ke mosaic
```

---

## 9. Tabel Parameter yang Bisa Di-Tuning (state sekarang)

| Parameter | Lokasi | Nilai sekarang | Fungsi |
|---|---|---|---|
| Inlier ratio (accept gate) | `combine()` baris 391 | `0.6` | Minimum "kesepakatan" match SIFT biar dipercaya. |
| Inlier ratio (eligibility kalibrasi) | `combine()` baris 405 | `0.7` | Minimum kesepakatan match buat BOLEH nentuin skala pixel↔meter (harus > gate accept). |
| `TRANSLATION_TOLERANCE_M` | `__check_gps_translation_sanity` | `10.0` meter | Toleransi selisih translasi SIFT vs GPS. |
| `MAX_BRIDGE_GAP` | `__init__` | `3` | Batas reject beruntun yang masih boleh "bridging". |
| `GPSDistanceFilter(threshold_m=...)` | `service.py`, di luar `Combiner.py` | `3.0` meter | Jarak minimum antar frame biar diterima masuk PROSES STITCHING sama sekali (admission-level, beda dari semua gate di atas yang Combiner-level). |

Kalo mau tuning ulang salah satu, INGET: ubah SATU variabel per percobaan, jangan borongan — gampang ke-bingung nentuin efek mana yang dari perubahan mana (pengalaman langsung sesi ini 😅).
