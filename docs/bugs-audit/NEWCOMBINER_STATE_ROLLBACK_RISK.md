# `newCombiner.py` — Incomplete State Rollback on Exception

> **Status**: Ditemuin saat review `src/newCombiner.py` vs `src/Combiner.py` (2026-11-05), BELUM difix. User udah tes `newCombiner.py` di base station dan berjalan aman — temuan ini BUKAN bug yang nongol di testing normal, tapi gap yang baru kena di kondisi tertentu (lihat §3). Dicatet buat dipertimbangkan sebelum masuk ke stitching service base station.

---

## 1. Konteks

`newCombiner.py` nambah kemampuan INCREMENTAL ke `Combiner` — `add_images(new_images, new_metadata)` nambah gambar baru ke sesi yang UDAH JALAN dan nerusin `combine()` cuma buat index baru, TANPA re-stitch dari frame 0 tiap kali. Ini nyelesain bug efisiensi O(N²) di model lama (tiap gambar baru masuk, stitching ulang dari awal).

Diff vs `Combiner.py` CUMA 3 perubahan fungsional (sisanya whitespace):
1. `__preprocess_images(raw_image, index_offset=0)` — tambah offset biar baca baris `dataMatrix` yang bener pas gambar nambah belakangan.
2. `add_images()` baru — append ke `dataMatrix`/`image_list`, lanjut `combine()` buat index baru, rollback `image_list`/`dataMatrix` kalo exception.
3. `create_mosaic()` dibungkus try/except, rollback serupa.

Perubahan ini KECIL dan AMAN buat jalur normal (gak ada exception) — temuan di bawah ini CUMA relevan buat jalur exception/error.

---

## 2. Masalah: rollback-nya gak lengkap

Di `combine(index)`, urutan mutasi state-nya:

```
1. self.last_acc_index = index              # baris ~462
   self._consecutive_rejections = 0
2. __advance_global_position()              # baris ~466 -- mutasi H_global_prev, H_rel_prev
3. _warp_images()                           # baris ~467 -- bisa throw (cv2)
4. ROIfeatherBlender._roi_feather_blend()   # baris ~475 -- assign self.result_image
```

`last_acc_index`, `_consecutive_rejections`, `H_global_prev`, `H_rel_prev` KE-UPDATE DULUAN (langkah 1-2), SEBELUM `self.result_image` ke-assign (langkah 4). Kalo exception kejadian di langkah 3 atau 4 — `_warp_images`/blending lempar error (misal gara-gara data gambar korup/dimensi aneh) — maka:

- `image_list`/`dataMatrix` DI-ROLLBACK (`add_images`'s except block: `del self.image_list[i:]`, `self.dataMatrix = self.dataMatrix[:i]`).
- TAPI `last_acc_index`, `_consecutive_rejections`, `H_global_prev`, `H_rel_prev` GAK IKUT DI-ROLLBACK — tetep nunjuk/merujuk ke state abis index yang gagal itu.

Akibatnya state jadi GAK KONSISTEN: array (`image_list`/`dataMatrix`) bilang "kita baru sampe index `i-1`", tapi variable lain bilang "kita udah di index `i`".

---

## 3. Kenapa belum pernah kena pas testing

Window buat kena bug ini SEMPIT — exception harus kejadian PERSIS di antara langkah 1 dan langkah 4 di atas (dalem `__advance_global_position`, `_warp_images`, atau `_roi_feather_blend`). SEMUA gate check lain (attitude drift, jumlah match, inlier ratio, GPS sanity) ada SEBELUM langkah 1 dan nge-`return` NORMAL (bukan exception) kalo gagal — jadi reject biasa gak pernah nyentuh window ini sama sekali.

cv2 operations di window itu (`warpPerspective`, blending) jarang throw KECUALI data gambar beneran korup/dimensi aneh (misal network loss yang sempet kebawa lolos reassembly tapi hasilnya JPEG rusak, atau estimasi transform ekstrem yang bikin ukuran canvas absurd). Testing manual sejauh ini kemungkinan gak pernah ngalamin kondisi se-ekstrem itu — makanya "aman-aman aja" itu VALID buat kondisi yang udah dites, tapi bukan jaminan buat SEMUA kondisi.

---

## 4. Skenario konkret yang bisa motong

- Gambar yang baru diterima KORUP sebagian (misal dari jalur UDP yang reassembly-nya gak sempurna, atau file ke-truncate) — `cv2.warpPerspective`/`cv2.cvtColor` bisa throw kalo array-nya aneh.
- Estimasi transform (`H_rel_3x3`) yang EKSTRIM (misal gara-gara match yang keliru tapi somehow lolos semua gate) bisa bikin `__compute_canvas_bounds` ngitung ukuran canvas yang gak masuk akal (kegedean/negatif) — `cv2.warpPerspective` bisa throw atau alokasi memory gagal.

Dua-duanya LANGKA di kondisi normal, TAPI di production (multi-UAV, 24/7, link yang bisa lossy — konteks "accelerated stitching" yang lagi dikerjain) peluangnya NAIK dibanding testing terkontrol.

---

## 5. Dampak kalo kena

- `last_acc_index` nunjuk row yang UDAH KE-TRUNCATE dari `dataMatrix` (kalo rollback motong sampe sebelum index itu) → `IndexError` di panggilan `combine()` BERIKUTNYA (nge-crash seluruh sesi UAV itu).
- ATAU, kalo index-nya masih valid secara angka tapi SEMANTIK-nya salah (nunjuk ke row yang sekarang keisi data gambar BEDA setelah batch berikutnya nambah), attitude-drift gate bakal ngitung drift terhadap frame yang SALAH — gak nge-crash, TAPI diam-diam ngasih hasil yang gak akurat (silent corruption, lebih susah ketauan dari crash).

---

## 6. Severity

- **Peluang kejadian**: RENDAH per-frame (butuh exception PAS di window sempit itu) — konsisten sama "udah dites, aman-aman aja".
- **Dampak kalo kejadian**: GEDE — crash sesi (kalo IndexError) ATAU silent data corruption (kalo index masih "valid" tapi salah semantik, ini yang lebih bahaya karena gak keliatan langsung).
- **Trade-off produksi**: makin lama sesi (makin banyak frame diproses), makin banyak kesempatan buat exception langka ini kejadian SEKALI doang — dan sekali kejadian, efeknya PERMANEN buat sisa sesi (state gak konsisten gak pulih sendiri). Worth difix sebelum masuk production, bukan "tunggu kejadian baru fix" — begitu kejadian, sesi UAV itu udah kadung rusak.

---

## 7. Fix yang diusulkan

**Prinsip kunci**: di seluruh codebase ini, `H_global_prev`/`H_rel_prev`/`result_image` SELALU di-REASSIGN (`self.x = new_value`), BUKAN di-mutasi in-place (gak ada `self.x[0,0] = ...` atau `self.x += ...` di manapun). Jadi nyimpen SNAPSHOT buat rollback bisa cukup simpen REFERENSI ke object lama (murah, `O(1)`), GAK PERLU `.copy()` array gede (`result_image` bisa jadi gede banget buat mosaic yang udah lama jalan — `.copy()` tiap frame bakal nambah overhead `O(ukuran_mosaic)` per frame, sayang kalo gak perlu).

```python
def add_images(self, new_images, new_metadata):
    old_length = len(self.image_list)
    self.dataMatrix = np.vstack([self.dataMatrix, new_metadata])
    processed = self.__preprocess_images(new_images, index_offset=old_length)
    self.image_list.extend(processed)

    for i in range(old_length, len(self.image_list)):
        # snapshot MURAH -- simpen referensi, bukan copy, karena combine()
        # selalu reassign 4 variable ini, gak pernah mutate in-place
        snapshot = (
            self.last_acc_index,
            self._consecutive_rejections,
            self.H_global_prev,
            self.H_rel_prev,
            self.result_image,
        )
        try:
            self.combine(i)
        except Exception:
            (self.last_acc_index, self._consecutive_rejections,
             self.H_global_prev, self.H_rel_prev, self.result_image) = snapshot
            del self.image_list[i:]
            self.dataMatrix = self.dataMatrix[:i]
            raise
```

**Kenapa snapshot per-index di DALAM loop** (bukan sekali di awal `add_images()`, sebelum loop): biar GRANULARITY rollback-nya KONSISTEN sama `image_list`/`dataMatrix` yang sekarang udah rollback PARSIAL (nyimpen frame yang UDAH SUKSES di batch yang sama, cuma buang yang GAGAL dan sisanya). Kalo snapshot diambil sekali di awal batch, restore bakal BUANG state frame-frame yang UDAH SUKSES juga (padahal `image_list`/`dataMatrix`-nya TETEP nyimpen mereka) — jadi malah bikin inkonsistensi BARU antara array vs state scalar, kebalik dari yang mau difix.

`create_mosaic()` butuh fix pattern yang sama (snapshot per-`i` di dalem loop-nya), karena punya try/except serupa.

---

## 8. Catatan sekunder (bukan bug di file ini, tapi gotcha pemanggilan)

`new_metadata` yang dioper ke `add_images()` HARUS pake origin X/Y yang SAMA kayak construction PERTAMA sesi itu. Kalo `new_metadata` dibikin lewat `build_data_matrix()` TANPA eksplisit nge-pass origin yang sama (default-nya ngambil dari baris pertama LIST YANG DIOPER, bukan origin sesi), X/Y batch baru bakal itung relatif ke titik yang BEDA dari batch sebelumnya → posisi mosaic LANGSUNG meleset/loncat begitu batch kedua masuk. Worth ditambahin docstring warning eksplisit di `add_images()`, atau divalidasi di dalem fungsinya (assert origin konsisten), biar gak kelewat pas integrasi.

---

## 9. Belum diputusin

Fix di §7 BELUM diterapkan ke `newCombiner.py` ataupun `src/Combiner.py` — nunggu keputusan user based on dokumen ini. Kalo mau diterapkan, scope-nya kecil (ganti 1 fungsi, `add_images()`, plus pattern serupa di `create_mosaic()`), gak nyentuh logic `combine()` sama sekali.
