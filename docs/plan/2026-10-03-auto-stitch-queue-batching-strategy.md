# Rencana: Auto-Stitch Queue/Batching Strategy

> **STANDALONE DOC** — ditulis di repo VISION-LIVESTITCH, dibawa ke session terpisah di repo base station (stitching service SEKARANG udah fully integrated di situ). Repo VISION-LIVESTITCH TIDAK bisa diakses dari session ini. Cross-ref ke dokumen VISION-LIVESTITCH lain (`MASTER_TODO_STITCHING_ENGINE.md`, `gcs_docs.md`, spec inlier-ratio, dll) di versi sebelumnya UDAH DIHAPUS/dijadiin catatan inline — kalo ada bagian yang kerasa ngegantung, itu konteks yang sengaja gak kebawa, bukan error.
>
> **Status**: Hasil brainstorming, BELUM ada keputusan final — SEMUA masih under consideration. §8 (ide batching: persistent Combiner instance, bukan reseed tiap batch) lagi dieksplor, belum diputusin, DAN nemuin bug efisiensi O(N²) yang independen dari isu ini (itu fakta/bug, bukan keputusan). Threading vs multiprocessing (§6) ada pertimbangan baru yang condong ke thread (lihat §8.2 poin 3), tapi BELUM final. Dokumen ini nyimpen diskusi + elaborasi biar gak ke-lost, BUKAN catatan keputusan.
>
> **Penting buat session baru ini**: karena stitching service udah fully integrated, file/fungsi yang disebut di dokumen ini (`service.py`, `src/Combiner.py`, nama method, nomor baris) itu REFERENSI DARI SEBELUM INTEGRASI (repo VISION-LIVESTITCH yang berdiri sendiri). Lokasi/nama persisnya di repo base station SEKARANG kemungkinan beda (nama file, struktur router, dll) — cari dulu padanannya di repo ini sebelum nerapin apapun dari dokumen ini secara literal.

---

## 1. Masalah yang mau diselesaiin

Dua masalah terkait yang teridentifikasi (dari planning VISION-LIVESTITCH sebelum integrasi):

**(a) Gambar terakhir "yatim"** — versi `run_stitching()` sebelum integrasi baca SEMUA gambar yang udah numpuk di sesi pas dia MULAI baca, bukan batch bersih kelipatan threshold auto-stitch (contoh lama: default `5`). Konsekuensi: gambar yang keterima PAS SETELAH satu batch stitch udah mulai jalan bisa nyangkut gak ke-stitch sama sekali sampe ada trigger lagi (auto berikutnya, atau manual trigger). Gak ada batas batch yang EKSPLISIT — cuma "baca semua yang ada sekarang." **Cek dulu**: apakah karakteristik ini masih ada di versi yang udah terintegrasi sekarang, atau udah berubah.

**(b) Karakteristik N-1 di algoritma chaining** — frame pertama di urutan gambar (`image_list[0]` di versi lama) jadi SEED doang, gak pernah di-feature-match/divalidasi independen (gak ada frame sebelumnya buat dibandingin). Ini emang desain algoritma chaining (pairwise sequential stitching), BUKAN bug queueing. TAPI kalo desain queue/batch baru motong-motong aliran gambar jadi batch-batch diskrit, gambar PERTAMA di tiap batch baru bakal kewarisan sifat "gak pernah divalidasi" ini juga. (a) dan (b) perlu didesain BARENG, gak bisa dipisah.

---

## 2. Ide awal user: producer/consumer dengan explicit signaling

Rough idea (verbatim dari diskusi): pas stitching jalan, bakal ada 2+ job — 1 job buat PROSES stitching (consumer), 1+ job BACKGROUND buat NGUMPULIN gambar (producer). Sisi server terus-terusan baca status background job itu: masih ngirim gambar, atau udah selesai. Begitu background job SELESAI ngirim semua gambar, server dapet SIGNAL, baru trigger stitch lagi.

Poin pentingnya: "implementasinya masih sama [kayak sekarang], tapi ini lebih ke PROPER SIGNALING dari queue/batch-nya." Beda dari sekarang (watchdog → nambahin ke `accepted_images` list → `_process_new_file` ngecek panjang list ngelewatin threshold atau belum, POLLING implisit), ide ini mau ada SIGNAL eksplisit "batch ini udah siap" dari sisi producer, bukan consumer yang nebak-nebak dari panjang list.

---

## 3. Kekhawatiran soal swarm/multi-UAV (bukan race condition, tapi resource contention)

User sempet mikir ini bakal jadi masalah "race condition" pas banyak UAV jalan bareng, tapi abis diskusi di-refine: BUKAN race condition (dicek bareng — `StitchingSession` per-UAV punya `threading.Lock` sendiri-sendiri, `self._state_lock`, jadi 2 UAV beda session emang GAK saling blocking di level lock, udah dikonfirmasi bener). Kekhawatiran sebenarnya: **gimana cara bagi kerjaan ke beberapa proses TANPA bikin mekanisme pembagian itu sendiri jadi bottleneck baru** di masa depan (pas integrasi GCS multi-UAV beneran).

**Temuan konkret dari baca kode `service.py` sekarang**: tiap kali `claim_stitch()` berhasil, server nge-spawn `threading.Thread` BARU, GAK ADA BATAS (lihat `_process_new_file` ~baris 186-189 dan endpoint `/stitch` ~baris 511). Gak ada ceiling concurrency GLOBAL — kalo N UAV trigger barengan, ya N thread jalan barengan, gak dibatasi sama sekali. Ini LATENT, belum kerasa sekarang karena baru 1 UAV/session yang aktif, tapi bakal jadi masalah nyata begitu multi-UAV beneran jalan.

---

## 4. Diskusi threading vs multiprocessing (GIL)

Hipotesis awal user: Python threading cuma ngasih CONCURRENCY, bukan PARALLELISM, buat kerjaan CPU-bound — jadi swarm context (yang butuh paralel beneran) harusnya pake multiprocessing, bukan threading. Ini BENER secara umum (GIL emang bikin thread Python murni gak bisa paralel buat kerjaan CPU-bound).

**Nuance yang ngubah kalkulasi buat codebase INI spesifik**: `cv2` dan `numpy` ngelepas GIL pas lagi ngerjain compute di level C. Jadi SIFT detect, feature matching, `warpPerspective`, dll — yang notabene HOT PATH `combine()` — itu GAK kena batasan GIL yang sama kayak loop Python murni. Thread yang lagi nunggu panggilan cv2/numpy SELESAI bisa dipake buat ngejalanin thread LAIN di core lain bener-bener BARENGAN.

Jadi pertanyaannya BUKAN "threading vs multiprocessing" secara abstrak, tapi: **berapa persen hot path `combine()` itu C-extension (gak kena GIL) vs Python murni (kena GIL)?** — ini yang diprofilling di §5.

---

## 5. Hasil profiling (data REAL, `output/uav_1/images`, 56 frame, `cProfile`)

```
detectAndCompute (SIFT)     1.370s  55.5%   cv2, GIL-released
imwrite                     0.306s  12.4%   cv2, GIL-released
_roi_feather_blend (kode sendiri) 0.253s  10.3%   numpy vectorized (*, +, **, boolean-mask indexing) -- GIL-released di level ufunc, cuma gak kepisah di cProfile sebagai fungsi C bernama terpisah kayak panggilan cv2.xxx()
warpPerspective             0.134s   5.4%   cv2, GIL-released
distanceTransform           0.058s   2.4%   cv2, GIL-released
combine() punya sendiri (glue code)  0.037s   1.5%   PYTHON MURNI (branching, bookkeeping list/dict)
sisanya                     ~0.3s   ~12%    cv2/numpy, tersebar
```

**Dicek langsung** `_roi_feather_blend` (`src/blending.py`) buat mastiin gak ada loop Python mentah yang nyembunyi di balik angka `0.253s` itu — TERNYATA gak ada, semuanya vectorized numpy (boolean-mask indexing, elementwise array math), gak ada `for` loop per-pixel.

**Kesimpulan**: overhead Python MURNI di SELURUH hot path cuma ~1.5% (`combine()`'s orchestration logic doang). Sisanya cv2/numpy C-call yang ngelepas GIL. Argumen naif "harus multiprocessing, Python gak bisa paralel buat CPU-bound" GAK langsung berlaku bersih di codebase spesifik ini.

---

## 6. Keputusan yang MASIH TERBUKA (lanjut sesi besok)

Profiling di atas BUKAN jawaban final — cuma ngilangin satu asumsi naif. Pertimbangan yang masih perlu dibahas:

- **Threading (worker pool berbasis thread)**: didukung sama temuan GIL-release di atas — compute beratnya emang udah paralel secara native via cv2/numpy. Gak ada overhead serialisasi (array gambar `uint8` gede-gede gak perlu di-pickle/lewat shared-memory buat nyebrang process boundary — itungannya lumayan kalo byte count-nya gede).
- **Multiprocessing (worker pool berbasis process)**: TETEP punya alasan sendiri yang gak ada hubungannya sama GIL — ISOLASI MEMORI antar session UAV (satu proses crash/leak gak nyeret proses lain), CRASH CONTAINMENT (satu job stitch yang crash gak matiin job UAV lain yang lagi jalan bareng), dan OS-level resource accounting/capping per worker lebih gampang di-enforce per-process ketimbang per-thread.
- **Keputusan inti yang BERLAKU TERLEPAS dari thread-vs-process**: butuh WORKER POOL YANG DIBATASI (fixed N concurrent stitch job, GLOBAL lintas semua session), ganti model sekarang yang nge-spawn thread BARU TANPA BATAS tiap trigger (§3). Ini yang paling kritis buat dibenerin duluan, independen dari jawaban thread-vs-process.

### 6.1 Perbandingan lebih detail (buat bahan review sebelum mutusin)

| Pertimbangan | Thread pool (`ThreadPoolExecutor`) | Process pool (`ProcessPoolExecutor`) |
|---|---|---|
| **Kompleksitas implementasi** | Drop-in dari model `threading.Thread` sekarang, minim perubahan. State (`session.accepted_images`, dll) tetep bisa diakses langsung lintas thread (sama-sama di 1 proses, sama memory space) — tinggal jaga lock yang udah ada. | Worker function HARUS bisa di-pickle, dan gambar/array yang dikirim ke worker ikut KEBAWA lewat IPC (pickled, nyebrang process boundary). Session state (`accepted_images`, `H_global_prev`, dll) TIDAK otomatis ke-share — harus didesain ulang: worker nerima PATH FILE + metadata (bukan array `numpy` mentah), biar gambar 4K (udah dibahas di plan TCP sender — gampang ratusan KB-beberapa MB per file) gak perlu di-pickle BOLAK-BALIK lewat IPC. Worker baca filenya sendiri dari disk. |
| **Serialisasi/overhead per-job** | Nol — semua state di memory yang sama. | Ada, TAPI bisa diminimalin kalo cuma kirim path+metadata (bukan array gambar) ke worker, dan worker baca file sendiri — hasil akhir (mosaic) juga ditulis LANGSUNG ke disk dari dalam worker (bukan dikirim balik lewat IPC sebagai array gede). |
| **Crash isolation** | Kalo ada crash C-level (misal OpenCV segfault — pernah kejadian secara umum pas gambar korup), bisa nge-crash SELURUH proses server, termasuk endpoint lain yang lagi aktif (video/telemetry/control). | Worker yang crash CUMA mati sendiri — SECARA TEORI gak nyeret proses server utama. **TAPI ada gotcha nyata**: `concurrent.futures.ProcessPoolExecutor` kalo salah satu worker-nya crash/mati gak normal, executor itu bisa masuk status `BrokenProcessPool` — SEMUA future/task yang lagi nunggu di executor itu ikut gagal, bukan cuma yang nyangkut di worker yang crash. Jadi "crash containment"-nya BUKAN otomatis gratis — perlu ada logic detect `BrokenProcessPool` + rebuild executor, kalo enggak satu crash bisa matiin seluruh kapasitas stitching sampe restart manual. |
| **Memory overhead** | Minimal — semua worker share library yang sama (cv2, numpy) di 1 proses. | Tiap proses worker punya copy sendiri dari library yang di-load (cv2 dkk) — overhead memory per-worker, kali lipat sama jumlah worker di pool. |
| **Dampak ke endpoint LAIN yang jalan bareng** | Kerjaan Python-murni (~1.5% hot path, bookkeeping/branching) tetep ikut kena GIL, bisa nambah micro-jitter dikit ke endpoint lain (video/telemetry/control) yang jalan di event loop yang sama — kemungkinan KECIL given cuma 1.5%, tapi gak nol. | Worker jalan di proses TERPISAH — gak ada kontensi GIL sama sekali ke proses server utama, termasuk buat bagian 1.5% Python-murni itu. |
| **Startup cost per worker** | Murah (thread spawn ringan). | Lebih mahal (process spawn/fork), TAPI gak masalah kalo pool-nya PERSISTENT (dibuat sekali di awal, dipake berkali-kali) — bukan spawn-ulang tiap job. |
| **Debugging** | Gampang — semua log/print/traceback langsung keliatan, bisa pake debugger biasa. | Perlu nambahin handling buat nangkep output/exception dari proses terpisah (logging per-worker, propagate exception balik ke proses utama). |

### 6.2 Rekomendasi sementara (buat didiskusiin, BUKAN keputusan final)

Kalo prioritas utamanya **kecepatan implementasi + compute udah kebukti paralel (profiling §5)** → thread pool dulu, upgrade ke process pool nanti kalo crash-containment beneran jadi masalah nyata di lapangan (YAGNI — jangan overengineer isolasi sebelum ada bukti butuh).

Kalo prioritas utamanya **robustness buat production multi-UAV** (1 job gagal gak boleh ganggu UAV lain SAMA SEKALI, termasuk video/telemetry/control yang jalan bareng) → process pool, TAPI WAJIB implementasi `BrokenProcessPool` handling dari awal (§6.1), bukan asumsi "proses terpisah = aman otomatis".

Hybrid yang mungkin masuk akal: mulai THREAD pool (cepat, simpel, sesuai temuan GIL-release), TAPI desain interface worker-nya (input: path+metadata, bukan array; output: ditulis ke disk, bukan dikembaliin lewat return value) SEAKAN-AKAN bakal pindah ke process pool nanti — biar migrasi ke process pool (kalo kebutuhan robustness muncul belakangan) gak perlu desain ulang dari nol, cuma ganti `ThreadPoolExecutor` jadi `ProcessPoolExecutor` + tambahin `BrokenProcessPool` handling.

---

## 7. Gimana ini nyambung ke ide producer/consumer (§2)

Reframe ide user jadi pola standar:

```
[per-session COLLECTOR, lightweight, I/O-bound]
   -- genuinely cocok threading/asyncio TERLEPAS dari isu GIL di atas,
      karena ini kerjaan I/O (nerima file), bukan CPU compute
        |
        v
[GLOBAL QUEUE -- lintas SEMUA session/UAV, bukan per-session kayak sekarang]
        |
        v
[WORKER POOL dibatasi, size N -- thread atau process, keputusan §6]
   -- narik dari GLOBAL queue itu
```

Desain ini LANGSUNG nyelesain masalah (a) gambar yatim: batas batch jadi EKSPLISIT (item di queue / sinyal "batch ready" dari collector), bukan lagi "baca apapun yang numpuk pas `run_stitching()` mulai baca."

Masalah (b) karakteristik N-1 — MASIH BELUM kesolved di sini, perlu dibahas terpisah: begitu aliran gambar kepotong jadi batch-batch diskrit beneran, gimana batch BARU nentuin frame pertamanya (seed baru yang gak pernah divalidasi LAGI, atau nyambung ke ekor batch sebelumnya buat tetep ada validasi N-1)? Ini keputusan desain yang kesambung langsung ke batch-boundary design di atas, belum diputusin.

---

## 8. Refinement (2026-10-04): persistent Combiner instance — resolve (b) SEKALIGUS bongkar bug efisiensi O(N²)

**Ide user**: jangan reset/reseed chain Combiner tiap batch. "Batch" BUKAN konsep pemotongan data, tapi konsep SCHEDULING/TRIGGER doang — background collector job (§2) nentuin KAPAN nge-flush gambar yang BENERAN udah selesai diterima ke `combine()`, dipanggil di Combiner instance yang SAMA, long-lived SEPANJANG sesi UAV itu (bukan dibikin ulang tiap batch). Scope collector job = CHUNK ukuran tetap (misal tiap N gambar), bukan seluruh flight — biar tetep progresif/periodik, bukan nunggu sampe landing.

Karena chain-nya gak pernah di-split, gak ada skenario "frame pertama batch baru yang butuh divalidasi ulang" — skenario itu gak exist dari sudut pandang Combiner. **Ini resolve masalah (b) lebih bersih** dibanding dua opsi yang sempet kebayang (keduanya nganggep tiap batch = run Combiner independen yang butuh keputusan "gimana seed-nya").

### 8.1 Temuan samping: bug efisiensi O(N²) yang UDAH ADA di `service.py` sekarang

Dicek ulang `run_stitching()` (`service.py`): tiap kali auto-stitch trigger, dia baca `list(session.accepted_images)` — SEMUA gambar dari awal sesi, bukan cuma yang baru — terus bikin `Combiner` instance BARU DARI NOL tiap kali (`my_combiner = Combiner.Combiner(images, dataMatrix, ...)`). Artinya tiap batch nambah N gambar baru, SELURUH riwayat dari frame 0 ikut di-re-stitch ulang.

Total kerjaan sepanjang 1 flight (threshold tiap N gambar, total M gambar): `N + 2N + 3N + ... + M ≈ O(M²/N)` — QUADRATIC, bukan linear. Makin panjang flight, makin parah, dan ini independen dari isu worker-pool/concurrency yang lagi didiskusiin di §4-6 (ini bug di LEVEL ALGORITMA, bukan soal paralel/gak paralel).

**Model persistent-Combiner (§8) otomatis ngebenerin ini juga**: kalo Combiner instance persist & `combine()` cuma dipanggil buat index BARU, total kerjaan sepanjang flight jadi `O(M)` — linear. Ini kemungkinan WIN TERBESAR yang available di sistem ini sekarang, LEBIH GEDE dampaknya dibanding worker-pool parallelism (§4-6) buat kasus single-UAV. Worker-pool tetep perlu buat kasus MULTI-UAV (biar N sesi bisa jalan bareng), tapi O(N²)→O(N) ini benefit yang kepake BAHKAN buat 1 UAV doang.

### 8.2 Tradeoff/resiko yang perlu diinget sebelum implementasi

1. **BUKAN cuma perubahan di orkestrasi/router — engine stitching-nya sendiri butuh API baru.** Versi sebelum integrasi: constructor-nya nerima SELURUH list gambar SEKALI doang, preprocess semua di awal, loop utamanya jalan sekali lalu selesai — GAK ADA method buat "tambahin gambar baru, lanjutin combine() dari index terakhir yang diproses". Cek source engine YANG SEKARANG (udah terintegrasi di repo ini) buat mastiin apakah karakteristik ini masih sama. Kalo masih, perlu nambah kemampuan baru (misal `add_images()` + lanjutin loop dari index terakhir) — ini perubahan ALGORITMA di engine, bukan cuma orkestrasi service-level.

2. **Server restart jadi lebih mahal.** Versi sebelum integrasi punya properti "session survive backend reload" (scan ulang folder dari disk) yang kerja PRESIS KARENA model-nya stateless (tiap stitch baca ulang dari folder dari nol) — **cek apakah base station yang sekarang masih punya properti ini**. Combiner yang stateful long-lived (state posisi mosaic cuma ada di memory) berarti server crash/restart di tengah flight BIKIN SEMUA STATE ITU ILANG — resume ya balik re-stitch dari 0 (ilang keuntungan §8.1), ATAU perlu checkpoint state ke disk berkala (requirement baru, belum didesain).

3. **Nge-tip keputusan thread-vs-process di §6.** Combiner yang stateful-long-lived butuh SESSION AFFINITY — batch dari UAV yang sama HARUS konsisten diproses di worker yang SAMA (state-nya gak gampang dipindah). Thread pool (shared memory, 1 proses) jauh lebih gampang nampung object stateful kayak gini dibanding process pool (object Python + state `cv2`/`numpy` gak gampang di-serialize/migrasi lintas proses pas lagi jalan tengah sesi). DAN kalo 1 worker PROCESS crash di model process-pool, bukan cuma 1 job yang ilang (kayak diasumsikan di §6.1), tapi SELURUH mosaic state sesi UAV itu ikut ilang — cost crash di process-pool jadi LEBIH GEDE dari yang dibayangin sebelumnya, bukan lebih kecil. Poin ini condong ke arah THREAD POOL, bukan netral lagi kayak kesimpulan §6.2 sebelumnya.

### 8.3 Status

**BELUM diputusin** — ide persistent Combiner (batch = scheduling bukan data-partition) masih under consideration, sama kayak semua opsi lain di dokumen ini. Dicatet di sini karena analisisnya (benefit O(N²)→O(N), tradeoff §8.2) udah cukup matang buat jadi bahan pertimbangan, BUKAN berarti udah final. Implementasi apapun (API baru di `Combiner.py`, checkpoint/resilience strategy, thread-vs-process) BELUM dimulai, nunggu keputusan.

---

## 9. Next steps (urutan buat sesi besok)

1. Putusin thread-pool vs process-pool buat worker side, pake pertimbangan §6 (GIL-released ops dominan, TAPI isolasi/crash-containment mungkin tetep lebih milih process).
2. Desain GLOBAL queue + worker pool dibatasi, ganti model spawn-thread-tanpa-batas sekarang (§3).
3. Desain sinyal "batch ready" eksplisit dari sisi collector, ganti polling implisit dari panjang `accepted_images` sekarang.
4. Selesain (b) karakteristik N-1 — gimana batch baru nyambung/mulai, bareng sama keputusan batch-boundary dari #2/#3.
5. Review ulang `threading.Lock` per-session (`self._state_lock`) — masih relevan begitu ada worker pool global, atau perannya ke-subsume sama pool itu sendiri?

---

## 10. Resources buat dibaca sebelum mutusin (dicari & diverifikasi 2026-10-04)

Dipilih yang LANGSUNG relevan sama 3 klaim di §5/§6.1 (GIL-release di numpy/opencv, thread vs process tradeoff, gotcha `BrokenProcessPool`) — bukan link generik.

**Soal GIL-release di numpy/opencv** (dasar dari temuan profiling §5):
- [How does the GIL release happen for drawing functions exposed to Python? — OpenCV Q&A Forum](https://answers.opencv.org/question/182036/how-does-the-gil-release-happen-for-drawing-functions-exposed-to-python/) — ini FORUM asli, jelasin mekanisme `PyAllowThreads`/`ERRWRAP2` yang OpenCV pake buat ngelepas GIL tiap panggilan fungsi dari Python. Paling relevan buat validasi langsung klaim "cv2 ngelepas GIL" yang jadi dasar argumen di dokumen ini.
- [NumPy vs the Global Interpreter Lock (GIL) — SuperFastPython](https://superfastpython.com/numpy-vs-gil/) — penjelasan kapan numpy ngelepas GIL (operasi array gede) vs kapan enggak (operasi kecil/overhead-nya gak worth). Catatan penting dari sini: operasi yang DURASINYA PENDEK (<10 microsecond) sebenernya RUGI kalo GIL dilepas (overhead lock/unlock lebih mahal dari kerjaannya sendiri) — worth dicek apakah ada bagian hot path `combine()` yang masuk kategori ini.
- [Hacker News thread — diskusi soal opencv & numpy GIL release](https://news.ycombinator.com/item?id=25406030) — diskusi forum lain, sudut pandang praktisi, bukan dokumentasi resmi.

**Soal perbandingan ThreadPoolExecutor vs ProcessPoolExecutor**:
- [concurrent.futures — Launching parallel tasks (dokumentasi resmi Python)](https://docs.python.org/3/library/concurrent.futures.html) — WAJIB baca, ini sumber kebenaran buat API-nya sendiri (termasuk behavior `BrokenProcessPool`).
- [ThreadPoolExecutor vs ProcessPoolExecutor in Python — SuperFastPython](https://superfastpython.com/threadpoolexecutor-vs-processpoolexecutor/) — perbandingan lebih dalam dari yang di §6.1, bagus buat silang-cek.

**Soal gotcha `BrokenProcessPool`** (yang dicatet di §6.1 sebagai resiko process-pool):
- [Why Your Python concurrent.futures Code Fails with 'BrokenProcessPool' — pythontutorials.net](https://www.pythontutorials.net/blog/all-example-concurrent-futures-code-is-failing-with-brokenprocesspool/) — penyebab umum: unhandled exception di worker, pickling error, worker kehabisan memory/di-kill OS, library C-extension yang emang gak didesain buat multi-process.
- [How to Debug Python's BrokenProcessPool — pythontutorials.net](https://www.pythontutorials.net/blog/finding-the-cause-of-a-brokenprocesspool-in-python-s-concurrent-futures/) — proses utama CUMA tau worker-nya mati gak normal, TAPI gak langsung tau KENAPA (child process-nya udah mati duluan sebelum bisa lapor alasan) — konfirmasi langsung soal "crash containment gak otomatis gratis" yang dicatet di §6.1.
