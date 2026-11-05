# Audit VISION-LIVESTITCH untuk dasar revisi

Tanggal: 26 Oktober 2026. Objek audit: working tree lokal, termasuk perubahan pengguna yang belum di-commit. Kode produksi tidak diubah dalam audit ini.

## Kesimpulan

Repositori memiliki komponen prototipe yang berguna, tetapi belum merupakan satu pipeline live mapping fixed-wing yang konsisten. Ada beberapa implementasi paralel: service batch berbasis folder, sender/receiver UDP, engine incremental, eksperimen GPS placement, dan simulasi bounding box. Fitur yang ada di satu jalur belum otomatis tersedia di jalur lainnya.

Dasar revisi yang paling menjanjikan adalah struktur `live_stitcher/`, setelah memperbaiki validasi transformasi, metadata, antrean, mask, dan isolasi UAV. Ambil kemampuan capture/telemetry dari `PROGRAM-SENDER1` serta arsip lokal dari `PROGRAM-NOVA/photo_capture.py` melalui adapter yang jelas. Pertahankan `src/Combiner.py` sebagai pembanding batch sementara. ODM menjadi jalur pemrosesan terpisah atas foto asli.

Status yang tepat saat ini: prototipe mosaik visual dengan beberapa jalur ingest; belum orthomosaic tervalidasi, belum multi-UAV incremental yang terisolasi, dan belum terintegrasi ODM.

## Cakupan dan batas bukti

- Inventaris dan parsing AST: **88 file Python, 19.936 baris**, tanpa syntax error. Inventaris menyimpan nama file, SHA-256, simbol dan nomor baris.
- Telaah mendalam: entrypoint root, service, receiver/sender, seluruh modul `live_stitcher`, Combiner/geometry/utilities/blending aktif, dan jalur capture/ground `PROGRAM-SENDER1`.
- Telaah pendukung: varian `PROGRAM-NOVA`, `PROGRAM-SENDER`, `src2`, simulasi live/bbox, `core_gmf`, legacy `main`/backup, skrip launcher, requirements, dokumentasi dan tes.
- Tidak semua cabang eksperimen/legacy dieksekusi. Tidak ada uji penerbangan, kamera USB, FC, radio, frontend base station eksternal, ODM, akurasi checkpoint, atau endurance ratusan frame.
- Dataset dan lingkungan virtual dikecualikan dari inventaris source. Foto lokal digunakan pada tes yang disebut di bawah. Dokumen PDF tidak dirender; dokumentasi Markdown dan source menjadi bukti utama.
- Screenshot progres adalah konteks target, bukan bukti bahwa seluruh fitur sudah terhubung dalam checkout ini.

Artefak pendukung berada di `docs/audit_2026_10_26/`: `inventory.json`, `checks.py`, `results.json`, `replay.py`, `replay.json`, dan preview replay. Skrip probe sengaja mereproduksi perilaku lama; bukan regression suite yang mengesahkan perilaku tersebut benar.

## Peta implementasi

| Area | Implementasi dan evaluasi | Keputusan revisi |
|---|---|---|
| Root `service.py` | API sesi, upload, watcher, WebSocket; membuat Combiner baru dari seluruh foto setiap job | Pertahankan kontrak API yang diperlukan, ganti eksekusi dengan job/worker yang memiliki state |
| `receiver_socket.py` | TCP per port, direktori `uav_1`/`uav_2`; bukan UDP | Jadikan adapter transport, identitas sumber eksplisit |
| Root sender | TCP panjang 4 byte + JPEG; sender kamera root tidak menambahkan telemetry | Satukan schema metadata dan mode replay/live |
| `src/Combiner.py` | Versi lokal baru: match foto ke mosaic, warp seluruh canvas, simpan intermediate setiap langkah | Baseline batch; bukan engine utama live |
| `src/geometry.py` | Sudut dalam derajat; matriks rotasi dipakai langsung untuk pixel warp tanpa kalibrasi intrinsik | Ganti model kamera dan kontrak unit |
| `src/blending.py` | ROI feather aktif; banyak blender eksperimen lain | Pertahankan API blending, benahi mask dan buang implementasi mati setelah migrasi |
| `src/utilities.py` | EXIF, import semua foto, default GPS nol | Parser terpusat, missing data eksplisit, streaming loader |
| `src/redundant_filter.py` | Eksperimen proxy kualitas dan keyframe; diimpor Combiner tetapi tidak dipanggil pada alur aktifnya | Evaluasi terpisah, jangan klaim metode sudah aktif |
| `live_stitcher/` | Pemisahan config, fitur, registrasi, canvas, receiver; incremental dengan cache 5 frame | Kandidat inti revisi |
| `PROGRAM_JUANG/PROGRAM-SENDER1` | UDP chunking, MAVLink, EXIF attitude, gate onboard/ground, monitor batch | Donor komponen telemetry, setelah bug diperbaiki |
| `PROGRAM_JUANG/PROGRAM-SENDER` | Varian sebelumnya; banyak kode serupa, launcher identik dengan SENDER1 | Arsipkan setelah migrasi fungsi yang dibutuhkan |
| `PROGRAM-NOVA` | Capture, EXIF injector, sender, receiver, ground, simulator; local archive opsional pada photo_capture | Donor arsip dan capture; sinkronisasi timestamp perlu diperbaiki |
| `SIMULASI LIVE` | Jalur TCP sederhana dan protokol JSON+bbox; bbox diproyeksikan dengan transformasi frame | Manfaatkan konsep proyeksi bbox, pisahkan overlay dari raster dasar |
| `src2` | GPS placement dan sequential stitching alternatif | Pembanding placement kasar; bukan georeferencing tervalidasi |
| `main`, `src/backup` | Legacy/backup; beberapa file identik dengan varian lain | Read-only baseline, jangan menjadi jalur runtime implisit |
| `core_gmf` | Ekstraksi fitur dengan multiprocessing; joiner tidak lengkap karena import `stitch_utils` hilang | Tidak layak dijadikan dependency runtime saat ini |
| `test`, `utils` | Dua tes otomatis GPS/import; lainnya script/demo; sejumlah file kosong | Bangun suite geometrik, protokol, state dan replay |
| Docs/launchers | Sebagian menjelaskan chain lama, bukan Combiner working tree yang baru | Dokumentasi per entrypoint dan versi |

Ada tiga protokol berbeda: TCP 4-byte length + JPEG; UDP `!III` + chunk; TCP bbox `!II` + JSON + JPEG. Ketiganya tidak saling kompatibel tanpa adapter. `OD` di beberapa folder berarti **Object Detection**, bukan OpenDroneMap.

## Temuan prioritas P1

P1 berarti perlu ditangani sebelum pipeline dipakai sebagai dasar pemetaan live yang dipercaya. Beberapa masalah hanya aktif pada entrypoint tertentu; jangan menggabungkan semua temuan seolah terjadi pada satu runtime.

### F01 — Unit dan tanda attitude rusak sepanjang jalur SENDER1

Bukti: `PROGRAM_JUANG/PROGRAM-SENDER1/sender.py:143` menggunakan `abs()` pada roll/pitch sebelum menyimpannya. `stitcher.py:342` mengubah sudut derajat menjadi radian, sedangkan `src/geometry.py:12` mengubah input derajat ke radian lagi.

Probe: yaw 90 derajat menjadi transformasi sekitar **1,5708 derajat**, bukan 90. Roll kiri/kanan juga tidak bisa dibedakan lagi setelah tanda dibuang.

Revisi: simpan sudut bertanda; `abs()` hanya untuk perbandingan gate. Gunakan unit eksplisit pada schema, konversi sekali, dokumentasikan ENU/NED dan transformasi body-camera. Tes +roll/-roll, +pitch/-pitch, yaw wrap dan mounting kamera.

### F02 — Koreksi orientasi bukan model proyeksi kamera yang terkalibrasi

Bukti: `src/geometry.py:6` membentuk R, memodifikasi elemen R, lalu menerapkannya langsung pada koordinat piksel. Tidak ada K/intrinsik, distorsi lensa, principal point, ataupun transformasi mounting.

Dampak: mengoreksi roll/pitch tidak dapat dibenarkan sebagai ortorektifikasi fisik. Sekadar memperbaiki unit F01 belum menyelesaikan model ini. Pada rotasi kamera murni, model pixel-domain memerlukan hubungan intrinsik seperti K R K^-1 sesuai konvensi; translasi dan relief tanah memerlukan model permukaan tambahan.

Revisi: kalibrasi kamera, undistort, definisikan kerangka koordinat, dan validasi proyeksi pada data sintetis sebelum menambahkan koreksi pose ke live pipeline. Jangan aktifkan koreksi lama untuk pose nonnol hanya karena parser telah diperbaiki.

### F03 — Pemaksaan rigid merusak hasil fit yang baru lolos RANSAC

Bukti: `live_stitcher/registrator.py:70` membagi komponen rotasi/skala dengan s, tetapi tidak melakukan fit ulang translasi ataupun menghitung ulang residual/inlier.

Probe 40 korespondensi tepat dengan skala 1,1 dan translasi: dilaporkan 40 inlier, tetapi transformasi yang dikembalikan mempunyai **median residual 41,76 px**, maksimum 66,42 px.

Revisi: pilih model berdasarkan asumsi akuisisi. Jika rigid diperlukan, lakukan fit rigid yang konsisten terhadap inlier dan hitung ulang residual setelah semua modifikasi. Jangan memaksa skala 1 saat tinggi AGL/zoom memang berubah. Tambahkan validasi distribusi inlier, residual, area footprint dan kondisi transformasi.

### F04 — State multi-UAV belum terisolasi di jalur incremental

Bukti: `live_stitcher/receiver.py:38` tidak membawa uav_id/mission_id; semua koneksi masuk ke satu queue. `live_stitcher/main.py:229` membuat satu LiveStitcher untuk seluruh koneksi.

Dampak: foto dua pesawat dapat diproses seperti satu urutan kamera. Berbeda dengan receiver root yang sudah memisahkan folder berdasarkan port; kedua pendekatan ini belum disatukan.

Revisi: router berdasarkan mission_id/uav_id/session epoch; satu state registrasi per stream; konsolidasi lintas-UAV dilakukan terpisah setelah georeferencing/matching. Attach/detach harus mendefinisikan retensi state dan kebutuhan relocalization.

### F05 — Reassembly UDP tidak membedakan sumber dan tidak memvalidasi indeks

Bukti: `PROGRAM_JUANG/PROGRAM-SENDER1/receiver.py:65` mengunci buffer hanya berdasarkan frame_id; alamat pengirim diterima tetapi tidak digunakan sebagai bagian key.

Probe: chunk indeks 0 dari UAV A dan indeks 1 dari UAV B dengan frame_id sama menghasilkan bytes campuran. Chunk total=1/index=999 memicu KeyError. Risiko campur sumber berlaku jika dua pengirim/restart stream berbagi receiver; port terpisah mengurangi kasus dua UAV tetapi tidak menyelesaikan session epoch dan paket lama.

Revisi: key `(mission_id, uav_id, stream_epoch, frame_id)`; validasi total/index/panjang, konsistensi header, batas bytes dan frame aktif, checksum frame, TTL monotonic serta cleanup yang berjalan saat tidak ada traffic. Chunk 60.000 byte juga berisiko IP fragmentation pada link dengan MTU biasa; ukuran perlu ditentukan melalui pengujian link.

### F06 — Antrean live tidak terbatas; overload menjadi delay dan RAM

Bukti: `live_stitcher/main.py:230` memakai Queue() tanpa kapasitas. Receiver men-decode gambar sebelum memasukkannya. Consumer satu thread memproses dan menyimpan mosaic.

Revisi: antrean bounded per UAV; arsip foto tidak bergantung pada lolos/drop queue live; drop policy mempertimbangkan overlap; tandai gap dan lakukan reacquisition bila perlu. Ukur queue age dan capture-to-display, bukan hanya waktu algoritma. Jangan menambahkan bounded queue lalu memanggil blocking `put()` di event loop; strategi admission harus nonblocking/async.

### F07 — Monitor ground memproses batch yang sama terus-menerus

Bukti: `PROGRAM_JUANG/PROGRAM-SENDER1/stitcher.py:477` menguji panjang seluruh `_accepted_imgs >= batch_size`; tidak ada watermark/version foto terakhir yang selesai diproses.

Probe memakai engine stub untuk mengisolasi scheduler: setelah satu job sukses, `should_stitch()` tetap True tanpa foto baru. Job berikutnya akan muncul lagi pada polling.

Revisi: snapshot input version, watermark completed/inflight, dirty flag dan idempotent job key. Kegagalan harus retry terkontrol. Untuk live, state incremental mempertahankan hasil sebelumnya.

### F08 — API stitching memblokir event loop dan watcher memakai loop lain

Bukti: `service.py:238` adalah async function tetapi memanggil import seluruh foto dan `create_mosaic()` sinkron. Route manual memasukkannya sebagai async BackgroundTask (`:463`). Watcher membuat thread + `asyncio.run` terpisah (`:132`, `:144`) untuk akses WebSocket yang dibuat server.

Dampak: jalur manual dapat menahan respons API selama komputasi. Jalur watcher memiliki risiko penggunaan transport WebSocket lintas event loop. Pengecekan `is_stitching` tanpa penguncian atomik juga membuka kemungkinan job ganda.

Revisi: executor/process worker khusus; semua pengiriman WebSocket melalui event loop aplikasi; atomic session/job state. Uji API responsif selama job dan dua trigger bersamaan.

### F09 — Metadata hilang berubah menjadi posisi yang tampak sah

Bukti: `PROGRAM-SENDER1/stitcher.py:616` membuat lat/lon progresif dan altitude 80 saat EXIF hilang. Parser default attitude 0. Foto tanpa EXIF dalam probe lolos gate dan memiliki posisi buatan.

Revisi: missing GPS/attitude = unknown, bukan nol atau posisi sintetis. Data sintetis hanya pada mode simulasi eksplisit dengan flag provenance. Jalur visual-only boleh berjalan tetapi harus berlabel unreferenced. Simpan foto tanpa GPS untuk rekonstruksi/peninjauan berikutnya; `live_stitcher/receiver.py:148` saat ini hanya mengarsip jika GPS ada.

### F10 — Metadata tidak terikat ke waktu exposure dan referensi altitude ambigu

Bukti: sender SENDER1 membaca attitude dan GPS melalui snapshot terpisah tanpa timestamp/age, menyimpan relative_alt ke GPSAltitude (`sender.py:195`). Nilai relative_alt terhadap home dilabeli AGL. `PROGRAM-NOVA/photo_capture.py` mengambil snapshot sebelum capture, bukan interpolasi berdasarkan exposure. Receiver incremental mencatat waktu penerimaan.

Dampak: telemetry yang sudah basi masih bisa lolos setelah FC berhenti mengirim; posisi/orientasi tidak selalu bersamaan dengan foto. Tinggi relatif home bukan tinggi tanah di bawah pesawat, dan tidak sama dengan MSL.

Revisi: capture_time, receive_time, telemetry_time/age, clock domain, uncertainty dan altitude datum eksplisit. Buffer telemetry dan interpolasi pada exposure timestamp; simpan signed attitude dan transformasi kamera. Eksperimen EXIF injector berbasis waktu file ditemukan tidak boleh dianggap geotag historis yang akurat.

### F11 — Path dari API belum dibatasi ke direktori sesi

Bukti: `service.py:40` memasukkan sessionId langsung ke path; `:436` memakai `session.image_folder / file.filename`; output_name juga langsung digunakan untuk baca/tulis.

Dampak: identifier/nama upload dengan traversal atau absolute path dapat keluar dari direktori yang dimaksud dengan hak akses proses. Ini temuan statis; audit tidak melakukan penulisan eksploitasi.

Revisi: ID tervalidasi; nama file internal/UUID; verifikasi resolved path berada di session root; batas upload dan validasi decode. Sesuaikan akses API dengan deployment. CORS bukan kontrol autentikasi.

## Temuan P2 dan kualitas desain

| ID | Bukti / masalah | Dampak dan revisi |
|---|---|---|
| F12 | `live_stitcher/redundancy_filter.py:19`, dipanggil sebelum registrasi di stitcher:191 | Posisi filter sudah maju ketika registrasi gagal. Probe: foto bagus berikutnya di posisi yang sama ditolak. Pisahkan evaluate/commit dan commit saat sukses |
| F13 | `live_stitcher/canvas.py:171` memakai warna hitam sebagai no-data | Probe: 1.600 piksel hitam valid pada area baru tidak dimasukkan. Warp validity mask terpisah; jangan infer coverage dari RGB |
| F14 | `canvas.py:488`, `:507`; `stitcher.py:431` | np.pad menyalin buffer; get_mosaic memindai semua bobot dan menghasilkan raster besar setiap frame. Klaim O(1) end-to-end tidak tepat. Dirty tiles, bbox tersimpan, preview rate limit, batas RAM |
| F15 | `stitcher.py:200`, timing sebelum get_mosaic; frame_times terus bertambah | accepted berarti lolos filter, bukan sukses stitch. Probe 12 accepted tetapi 11 mosaic returns. Pisahkan admitted/registered/rendered/rejected/failed; ukur total wall time, bukan sukses saja |
| F16 | `live_stitcher/gps_utils.py:152`, stitcher:266 dan config.gsd_default | East/North langsung dianggap sumbu pixel; north positif menjadi y positif, tanpa rotasi kamera. GSD disebut pada resolusi downsample tetapi dikali ds lagi; estimate tidak diperbarui. Definisikan GSD native/working dan earth-to-image, beri fallback full-frame bila ROI gagal |
| F17 | `heading_corrector.py`, `stitcher.py:504` | Course GPS bukan selalu yaw kamera, khususnya crab/crosswind. Referensi heading pertama bukan north-up absolut; rotasi fixed-size memotong gambar. reset() stitcher tidak mereset heading_corrector. Jadikan prior opsional, mask terpisah, reset seluruh state |
| F18 | `feature_engine.py:86`, cache 5 frame | Memilih kandidat berdasarkan jumlah match sebelum validasi geometri; tidak mencoba kandidat berikutnya jika terbaik gagal. Ranking berdasarkan inlier/residual/spread; simpan keyframe spasial untuk reacquisition |
| F19 | `src/Combiner.py:239` dan warp/intermediate | SIFT pada seluruh mosaic, warp canvas penuh, visualisasi dan PNG setiap frame. Biaya bertambah; pencocokan terhadap mosaic tidak menjamin drift hilang. Pisahkan logging/debug I/O dan gunakan keyframe asli |
| F20 | `src/Combiner.py:125` | Mask RANSAC diabaikan; affine non-None diterima tanpa syarat inlier/residual/footprint/canvas cap. Tambah geometric acceptance gate sebelum alokasi |
| F21 | `src/Combiner.py` preprocessing mengubah elemen list input menjadi None | Fallback OpenCV pada ground bisa menerima list yang telah dirusak ketika Combiner gagal. Jangan mutasi input milik caller; kelola ownership eksplisit |
| F22 | `PROGRAM-SENDER1/stitcher.py:227` | Overlay GPS memakai skala default 0,5 m/px dan origin foto pertama sebagai pojok mosaic; crop/warp tidak dipropagasikan. Gunakan H frame + canvas offset untuk bbox; geo overlay memerlukan geotransform yang benar |
| F23 | Receiver menulis langsung .jpg; monitor ground menandai processed sebelum decode/metadata | File parsial dapat dianggap sudah selesai dan tidak dicoba ulang. Tulis .part lalu rename atomik; event committed; output mosaic juga atomic publish |
| F24 | `service.py` hanya on_created + ambang N; watermark diperbarui sebelum sukses | Foto terakhir yang datang ketika job aktif bisa tertinggal tanpa event baru. Tambahkan drain/dirty state dan flush akhir misi |
| F25 | `LIVE_STITCHING_START.sh`, receiver root | Launcher tidak membuat sesi, meneruskan --session-id/--port yang tidak diparse receiver, sedangkan receiver memakai uav_1/uav_2. Pada instalasi baru monitoring tidak otomatis aktif; launcher hanya GNOME/Linux. Satukan config dan smoke test cold start |
| F26 | `src/ImageMosaic.py:78` | Memanggil createMosaic(), tetapi Combiner aktif menyediakan create_mosaic(). Entry point legacy tidak kompatibel; perbaiki atau tandai deprecated |
| F27 | `core_gmf/image_corrector.py:6` | Import `core_gmf.stitch_utils` gagal; file tidak ada. Probe mengonfirmasi ModuleNotFoundError. Jangan menyebut engine multiprocessing ini siap pakai |
| F28 | Root requirements dan konfigurasi | `geopy` diimpor util tetapi tidak ada di root requirements; scipy/pytest duplikat; versi dominan lower bounds; CLI default berbeda dari dataclass config. `pip check` lulus pada env sekarang tidak membuktikan clean install lengkap |
| F29 | `live_stitcher/main.py:277` | Semua exception consumer ditelan seperti queue timeout, sehingga bug bisa tersembunyi. Tangkap queue.Empty terpisah; log error/status; stop event, drain policy dan join saat shutdown |
| F30 | `live_stitcher/receiver.py:117` | Header >50MB di-continue tanpa menguras payload/menutup koneksi, sehingga framing TCP dapat desinkron. Tutup koneksi invalid; deadline read dan limit pixel decode |
| F31 | `src/blending.py`, metode tidak aktif | adaptive_weight_map memanggil gaussian_weight_map yang tidak ada; incrementalFusion menyimpan warna winner lalu membagi dengan weight lagi. Jangan mempromosikan helper ini tanpa tes |
| F32 | Canvas multiband/feather | Distance transform pada bounding box overlap bisa kehilangan batas mask asli; pembobotan multiband asimetris; baris/kolom ganjil di luar ukuran even tidak diblend. Uji mask dan seam numerik. Blending tidak menyelesaikan salah registrasi/parallax |
| F33 | `src/redundant_filter.py` | Fungsi bernama BRISQUE adalah proxy statistik tanpa model regresi, bukan skor BRISQUE standar; input uniform dapat berskor nol. Keyframe pendek bisa tidak terpilih saat sample_indices <2. Validasi terhadap blur/overlap; jangan klaim reproduksi paper tanpa pembandingan |
| F34 | Seluruh output aktif | PNG mosaic tanpa CRS/geotransform/checkpoint bukan orthophoto tergeoreferensi. Simpan transform chain/crop offset; pisahkan live preview dan peta final |
| F35 | Repo/dokumentasi | Banyak implementasi duplikat, manipulasi sys.path dan import bare `config/utilities/blending`; module resolution bergantung entrypoint/CWD. `.gitignore` memuat byte NUL. Buat package tunggal, import relatif, executable CLI dan ignore bersih |

## Hasil verifikasi yang dijalankan

Lingkungan: Python 3.13.7, OpenCV 4.13.0, NumPy 2.5.0 dari `.venv` yang tersedia. Tidak memasang dependency baru.

1. AST seluruh inventaris: 88 file lolos syntax parse. Ini bukan bukti semua import/fitur berfungsi.
2. `python -m pip check`: tidak ada broken requirements pada lingkungan terpasang.
3. Dua tes GPS/import ditargetkan: 2 passed, 4,17 detik. Kemudian full `pytest -q -p no:cacheprovider`: **2 passed, 19,24 detik**. Jumlah tetap dua; coverage saat ini sangat terbatas.
4. Probe offline: mismatch unit, postfit rigid residual, mask hitam, filter commit sebelum registrasi, UDP collision/invalid index, scheduler batch berulang, GPS sintetis, import service dan missing core_gmf. Hasil lengkap ada di results.json.
5. Replay **12 foto pertama secara leksikografis** dari `dataset/test_orthomosaics_42imgs` menggunakan dataclass default, bukan CLI defaults. **11 pemanggilan mengembalikan mosaic termasuk inisialisasi**, satu gagal estimasi affine. Tidak dilakukan tuning untuk mempercantik hasil.

| Metrik replay | Hasil |
|---|---:|
| Median process_file wall time | 1.181 ms |
| P95 process_file wall time | 1.479 ms |
| Canvas akhir internal | 3.151 × 2.841 px |
| Dua buffer canvas float32 saja | 143.231.856 byte (~136,6 MiB) |
| Statistik accepted internal | 12 |
| Pengembalian mosaic aktual | 11 |

Wall time replay mencakup load/EXIF/process/get_mosaic; tidak mencakup radio, antrean jaringan, UI atau PNG publish per frame. Buffers bytes bukan total peak RSS. Hasil ini bukan benchmark hardware UAV, bukan pembuktian sustained FPS, dan bukan pengukuran akurasi geografis. Pada konfigurasi dan beban uji ini, belum ada dasar mengklaim mampu mengikuti input kontinu 2 FPS tanpa backlog.

Preview hasil menunjukkan ketidakselarasan dan ghosting terutama pada area sungai/tanah, serta batas exposure. Penyebab spesifik tiap seam belum dapat dipastikan hanya dari gambar; parallax, model transformasi, dan koreksi heading perlu eksperimen ablation. File timing lama tidak dibandingkan sebagai speedup karena dataset/config/versinya tidak tercatat konsisten dan perhitungan total Combiner mengecualikan preprocessing sementara tabel menjumlahkannya.

![Preview replay 12 foto](<D:/KULIAH/BIMA - SWARM/VISION-LIVESTITCH/docs/audit_2026_10_26/replay_preview.jpg>)

## Kesesuaian dengan progres slide

| Target slide | Bukti dalam checkout | Evaluasi |
|---|---|---|
| UDP receiver per UAV | Receiver UDP terdapat di folder PROGRAM; root/incremental memakai TCP | Ada komponen UDP, isolasi sumber dan runtime manager perlu revisi |
| Attach/detach multi-UAV di base station | API root mengelola sesi/monitoring; tidak ditemukan implementasi frontend/modal yang disebut | Integrasi frontend eksternal tidak dapat diverifikasi di repo ini |
| Perbaikan drift sequence panjang | Combiner lokal match-to-mosaic; incremental rigid + GPS heading | Ada upaya, tetapi klaim eliminasi drift tidak terbukti dan ada bug geometrik |
| Attitude drift gate | Gate absolute roll/pitch di SENDER1; incremental hanya menerima lat/lon/alt | Bukan delta attitude roll/pitch/yaw end-to-end yang sama dengan slide |
| GPS translation fallback | GPS placement di src2; incremental memakai GPS untuk filter/ROI/heading, gagal visual tetap return None | Belum ditemukan fallback translasi GPS aktif pada dua engine utama |
| Parallel multi-UAV live | Async receiver/thread; multiprocessing eksperimen core_gmf | Belum ada worker pool live per UAV yang terintegrasi |
| ODM SDK/API + serving | Tidak ditemukan klien/job NodeODM/PyODM pada source Python yang dipindai | Pekerjaan baru, bukan fitur yang sudah berjalan |

## Rancangan revisi yang disarankan

```text
Capture + telemetry buffer
  -> immutable original archive + frame manifest
  -> preview transport adapter (UDP/TCP)
  -> validate/reassemble/deduplicate
  -> route by mission / UAV / stream epoch
  -> bounded live queue per UAV
  -> quality gate + candidate keyframes
  -> feature worker pool
  -> ordered registration/state commit per UAV
  -> tiled mosaic + independent overlays
  -> rate-limited preview/events to base station

Original archive + mission manifest
  -> postflight dataset validation
  -> ODM job adapter / NodeODM
  -> orthophoto assets + quality report
  -> final map layer
```

Usulan schema frame: schema_version, mission_id, uav_id, stream_epoch, frame_id, capture_timestamp, clock_domain, receive_timestamp, image dimensions/encoding/checksum, camera_id/calibration_id, GPS dan uncertainty, signed camera/body attitude dengan unit/convention, telemetry_age_ms, altitude dengan datum, simulation flag. Unknown dinyatakan null/status, tidak menjadi koordinat buatan.

Metadata output live: frame status/reason, H terhadap keyframe/mosaic, inlier count/ratio/spread, residual, footprint, working GSD, canvas origin/crop transform, queue age, time tiap tahap, coverage dan confidence. Georeferencing yang belum valid harus terlihat sebagai status, bukan disembunyikan.

API revisi yang diusulkan (bukan endpoint yang sudah ada): create mission; attach/detach UAV stream; stream health; latest preview/tiles; finalize mission; submit mapping job; job status/cancel/retry; list/download mapping assets. Status dan manifest persisten agar restart tidak menghilangkan hubungan input-output.

## Tahapan revisi dan kriteria penerimaan

| Tahap | Lingkup | Kriteria selesai |
|---|---|---|
| R0: Bekukan baseline | Manifest file/config/dataset, entrypoint tunggal, inventaris legacy | Replay lama bisa diulang tanpa menimpa data |
| R1: Benarkan data | Unit/tanda pose, missing GPS, capture timestamp, altitude datum, raw archive, atomic file commit | Uji metadata roundtrip; data sintetis terpisah; foto tidak hilang akibat gate live |
| R2: Benarkan geometri | Rigid fit, residual, mask, keyframe commit/retry, reset, model kamera | Transformasi sintetis known-truth, penolakan outlier, dark-pixel coverage, reconnect tidak merusak state |
| R3: Runtime satu UAV | Incremental worker, bounded queue, health/error reporting, atomic output | API tetap responsif; overload/restart/end-of-mission terdefinisi; queue age terukur |
| R4: Multi-UAV | Isolasi source/session, reassembler tahan reorder/loss, scheduling | Dua stream dengan frame ID sama tidak mencampur state/payload; reconnect dan paket lama diuji |
| R5: ODM | Job adapter, original dataset, hasil geospatial dan layer final | Job dapat ditelusuri, retry/cancel, output CRS/datum tercatat, checkpoint independen bila mengklaim akurasi |
| R6: Optimasi | Profiling, feature pool, tiles, tuning preview/fps | P95 latency, peak RSS, throughput dan registration success memenuhi target perangkat yang disepakati |

Prioritas bukan langsung menambah jumlah proses. Perbaiki R1–R3 dahulu; paralelisasi state yang masih salah hanya membuat kesalahan lebih cepat. Feature extraction dapat paralel, tetapi state commit per UAV harus berurutan. ODM dan live worker perlu resource budget terpisah.

Pengujian wajib revisi: translasi/rotasi/skala diketahui; roll/pitch bertanda; yaw wrap; perubahan AGL; foto blur dan fitur berulang; frame tanpa EXIF; packet loss/reorder/duplicate; collision antar-UAV; restart sender; queue overload; tidak ada foto baru; akhir misi di bawah batch threshold; path traversal rejection; shutdown saat job aktif; long-sequence replay dengan manifest yang sama. Bandingkan hasil live pada titik yang bisa ditelusuri ke foto asli; gunakan checkpoint independen untuk akurasi absolut, bukan menganggap ODM ground truth otomatis.

## Yang perlu dipertahankan

Struktur modular incremental, cache descriptor terbatas, transform chaining terhadap frame cache yang benar, ROI warping, callback/event progress, arsip lokal pada capture NOVA, gagasan gate kualitas di sisi ground, serta proyeksi bbox melalui H pada simulator merupakan fondasi yang layak. Revisi sebaiknya mengonsolidasikan fondasi tersebut dengan kontrak data dan pengujian yang konsisten, lalu menambahkan pemetaan ODM sebagai layanan terpisah.
