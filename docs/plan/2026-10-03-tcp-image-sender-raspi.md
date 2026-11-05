# Rencana: TCP Image Sender buat UAV-side (Raspi)

> **Status**: ✅ Opsi **(B) dipilih buat langkah SELANJUTNYA** (2026-10-03 malam) — instrumentasi loss-rate counter dulu, BELUM migrasi ke TCP. Alasan: ada uji terbang besok, jadi bisa diukur loss-rate di kondisi flight ASLI dulu sebelum mutusin (A) atau tetep UDP — based on ANGKA, bukan "belum pernah ngalamin" yang observasinya belum tentu cover kondisi terberat (§1, §7). IMPLEMENTASI BELUM DITULIS — ini to-do buat sebelum uji terbang besok.
> Cross-ref: `docs/MASTER_TODO_STITCHING_ENGINE.md` (section "To-Do Terkait, Beda Scope: Integrasi ke GCS", item 3 UDP vs TCP), `docs/plan/2026-10-03-auto-stitch-queue-batching-strategy.md` (sibling plan, sesi yang sama).

---

## 1. Konteks

**KOREKSI dari draft awal dokumen ini**: awalnya dikira `PROGRAM_JUANG/PROGRAM-SENDER1/sender.py` (UDP) itu buat pipeline VIDEO/TELEMETRY yang TERPISAH dari gambar stitching. SALAH. Udah dibaca langsung kodenya: `sender.py` → `FrameSender` ngirim ke `IMAGE_PORT=5600` (UDP), `receiver.py` sisi GCS nerima & reassemble lewat `FrameReassembler`, lalu `_save_frame()` nulis LANGSUNG ke `sessions/<session>/images/` — PERSIS folder yang sama yang dibaca `service.py`'s watchdog buat auto-stitch. Jadi ini channel yang UDAH LIVE DIPAKE anak buah user buat ngirim gambar stitching, BUKAN channel video terpisah yang perlu "ditambahin" TCP sender baru di sebelahnya.

**Konfirmasi dari user**: channel UDP ini UDAH pernah dipake buat akuisisi data pas terbang beneran (bukan cuma ground test), dan SEJAUH INI belum pernah ngalamin data loss yang kecatet. TAPI belum dikonfirmasi apakah akuisisi itu kejadian pas pesawat lagi aktif manuver (kondisi RF paling berat: jarak berubah, orientasi antena berubah) atau pas kondisi terbang yang lebih tenang/akuisisi-doang. Ini variabel penting yang masih OPEN (lihat §7).

**Fakta teknis baru yang relevan buat resiko loss**: kamera di `sender.py` itu 4K (`CAMERA_WIDTH=4096`, `CAMERA_HEIGHT=3072`, `config.py`), `CHUNK_SIZE=60000` byte per chunk UDP. Satu JPEG (gampang ratusan KB sampe beberapa MB di quality 85) kepecah jadi BELASAN-PULUHAN chunk per frame. Peluang MINIMAL 1 chunk ilang itu COMPOUND dari banyaknya chunk (`1 - (1-p)^n_chunks`, `p`=peluang loss per-paket) — beda dari kasus video yang biasanya per-frame jauh lebih kecil/dikit chunk. Jadi "belum pernah loss" itu data REAL dan harus dihargai, tapi belum tentu representative buat SEMUA kondisi (misal manuver tajam, jarak jauh, cuaca beda).

Buat gambar STITCHING, 1 gambar ilang BUKAN hal sepele kayak 1 frame video ilang — gambar-nya udah di-filter jarang (`GPSDistanceFilter`, ~2-3m antar frame), tiap satu berharga, 1 ilang = gap di chain stitching (kategori masalah yang sama kayak chain-bridging yang baru dibenerin malem ini, cuma sumbernya jaringan bukan gate).

---

## 2. Keputusan arah (REVISI — bukan "tambah channel baru", tapi pertimbangan MIGRASI channel yang udah live)

Opsi yang dipertimbangin, bukan cuma "deploy TCP" doang:

**(A) Migrasi channel gambar stitching ini (port 5600) dari UDP ke TCP**, pake protokol yang sama kayak `receiver_socket.py` yang udah ada di repo ini (`[4-byte big-endian length header][raw JPEG bytes]`, `asyncio.start_server`). Sisi receiver GCS udah ADA dan udah TCP — gak perlu ditulis ulang, tinggal sender-nya (`sender.py`) yang perlu versi TCP.

**(B) Tetep UDP buat sekarang, tambahin instrumentasi loss-rate dulu** — `FrameReassembler.cleanup_stale()` (`receiver.py`) UDAH ngeprint warning tiap frame timeout/incomplete, TAPI gak ada COUNTER/metric yang direkap (cuma log line, ilang abis discroll). Tambahin counter sederhana + persentase loss per sesi, biar keputusan migrasi nanti based on ANGKA REAL dari kondisi flight asli (termasuk manuver), bukan "belum pernah ngalamin" yang observasinya belum tentu cover kondisi terberat.

**✅ DIPUTUSIN: (B) dulu**, sebelum uji terbang besok. (A) nyusul based on hasil pengukuran (B), bukan diputusin sekarang.

---

## 3. 🔧 TO-DO SEBELUM UJI TERBANG BESOK (opsi B — scope implementasi)

Kecil, GAK nyentuh `sender.py` sama sekali — cuma `receiver.py`:

1. Tambah counter di `FrameReassembler` (`receiver.py`): total frame_id yang MASUK (dapet minimal 1 chunk) vs total frame yang BERHASIL di-reassemble lengkap vs total yang di-drop via `cleanup_stale()` (timeout/incomplete).
2. Rekap persentase loss (`dropped / total_masuk`) di `status_printer()` yang UDAH ADA (`receiver.py`, jalan tiap 15 detik) — tambahin 1 baris ke log yang udah jalan, gak bikin mekanisme baru.
3. Pas uji terbang besok: jalanin kayak biasa, abis selesai cek log `status_printer` buat liat angka loss-rate real. Kalo ada beberapa fase terbang (tenang vs manuver aktif), idealnya dicatet KAPAN tiap fase itu biar loss-rate bisa dikorelasiin ke kondisi RF (§7 poin paling penting).
4. Angka ini yang jadi dasar keputusan (A) migrasi ke TCP atau tetep UDP — bukan diputusin dari "perasaan belum pernah loss" lagi.

---

## 4. Kenapa TCP masuk akal KALO opsi (A) nanti jadi dipilih (ringkesan elaborasi sesi ini, buat referensi lanjutan — BUKAN to-do sekarang)

- **Reliability "gratis"** dari TCP sendiri (retransmit + ordering) — gak perlu nulis ulang logic reassembly/chunk/timeout kayak yang ada di `sender.py`/`receiver.py` UDP sekarang (`FrameReassembler`, dll). Kode sender jadi JAUH lebih simpel.
- **Match langsung** sama protokol `receiver_socket.py` yang UDAH ADA di repo ini — gak perlu ubah receiver sama sekali, tinggal `sender.py` versi TCP yang connect ke situ.
- Attitude/GPS EXIF embedding di `sender.py` (`embed_flight_metadata_exif`) TETEP DIPAKE APA ADANYA — itu bagian yang gak ada hubungannya sama UDP/TCP, cuma transport layer-nya doang yang beda.

---

## 5. Resiko REAL yang perlu ditangani (BUKAN alasan buat gak pake TCP, tapi gak boleh didiemin)

1. **Link RF ke drone bisa flicker/disconnect.** TCP asumsi koneksi stabil. Kalo link putus di tengah kirim → butuh RECONNECT-WITH-BACKOFF logic di sender (gak otomatis dari TCP, harus ditulis manual).
2. **Head-of-line blocking.** Kalo link lagi jelek, `send()` bisa nge-block/retry. UDP gak punya masalah ini (fire-and-forget, langsung lanjut) — TAPI itu "kelebihan" UDP cuma kepake kalo loss-nya OK buat didiemin, yang UDAH DIPUTUSIN TIDAK OK buat gambar stitching (lihat §1). Jadi ini bukan argumen balik ke UDP, cuma hal yang perlu di-mitigasi di sisi TCP.
3. **Backlog/memory growth.** Kalo `send()` lagi ke-block sementara kamera tetep capture terus, gambar bisa numpuk di memory Raspi kalo gak di-handle. Mitigasi: sender jalan di THREAD/QUEUE TERPISAH dari capture loop (biar capture gak ikut ke-block nunggu network), PLUS bounded queue (biar gak numpuk tak terbatas kalo disconnect lama).
4. **Drop policy pas queue penuh.** Kalo bounded queue di atas penuh (disconnect lama), keputusan "buang gambar BARU (reject capture baru) atau buang gambar LAMA (drop yang udah ngantri, prioritasin yang terbaru)" itu KEPUTUSAN DESAIN sendiri, belum diputusin — jangan didiemin jadi default behavior yang gak disengaja.

---

## 6. Scope implementasi KALO opsi (A) dipilih (perkiraan, BELUM ditulis)

Kecil — bukan rewrite besar, ~30-40 baris based on protokol `receiver_socket.py` yang udah ada, MENGGANTI `FrameSender` class di `sender.py` (bukan nambah di sebelahnya):

1. Socket TCP client di Raspi, connect ke host:port GCS (`receiver_socket.py`).
2. Kirim pake protokol yang sama kayak `receiver_socket.py` expect: `struct.pack("!I", len(img_bytes))` + raw bytes. GANTI `FrameSender.send()` yang sekarang fragmentasi-UDP.
3. Sender loop jalan di thread/queue terpisah dari capture loop (§5 poin 3) — biar capture loop (`cap.read()`, 0.5s interval) gak ikut ke-block nunggu network.
4. Bounded queue + drop policy eksplisit (§5 poin 4) — PERLU DIPUTUSIN dulu sebelum nulis kode, bukan di-assume pas implementasi.
5. Reconnect-with-backoff kalo koneksi putus (§5 poin 1).
6. `embed_flight_metadata_exif` dan attitude gate (`FlightDataMonitor`) di `sender.py` TETEP SAMA, gak kesentuh — cuma `FrameSender`/transport-nya yang diganti.

**Kalo opsi (B) yang dipilih duluan**: scope-nya lebih kecil lagi — cuma nambah counter di `FrameReassembler.cleanup_stale()` (`receiver.py`) buat ngerekap jumlah frame timeout/total frame per sesi, print ringkasan di `status_printer()` yang udah ada. Gak nyentuh `sender.py` sama sekali.

---

## 7. Belum diputusin / lanjut sesi depan

- **PALING PENTING**: konfirmasi ke anak buah — akuisisi data yang "belum pernah loss" itu kejadian pas kondisi RF PALING BERAT (manuver aktif, jarak jauh) atau pas kondisi lebih tenang/sekedar akuisisi? Ini nentuin apa opsi (A) langsung jalan atau (B) dulu (instrumentasi) sebelum migrasi.
- Kalo (A) dipilih: drop policy pas queue penuh (buang baru vs buang lama, §5 poin 4), ukuran bounded queue, reconnect backoff strategy (fixed interval? exponential?).
- Kalo (B) dipilih duluan: berapa lama/berapa sesi data loss-rate perlu dikumpulin sebelum cukup buat mutusin pindah ke (A) atau tetep UDP.
- Baik (A) maupun (B), idealnya diukur di LINK RF ASLI (bukan cuma ground test) sebelum jadi keputusan final buat production.
