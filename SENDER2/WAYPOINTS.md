# Pemicu 5 waypoint — satu fixed-wing, UDP

Implementasi tahap ini khusus **UAV1 (fixed-wing)**. Masukkan tepat lima pasangan latitude/longitude. Saat posisi GPS terbaru UAV1 berada pada jarak horizontal **≤20 meter** dari salah satu titik, program mengantrekan titik tersebut untuk dikirim melalui UDP ke ground. Semua titik diperiksa, tanpa kewajiban urutan WP1→WP5. Jika beberapa titik sekaligus berada dalam radius, semuanya dapat terpicu.

Program berjalan terpisah dari stitching. Di Raspberry Pi, `waypoints.py` membaca FC melalui MAVLink dan mengirim event. Di ground, `tools/waypoint_receiver.py` merupakan penerima contoh yang menyimpan event dan membalas ACK. Integrasi aplikasi base station serta penugasan copter belum dilakukan; program ini tidak mengubah mission, mode, arming, atau navigasi wahana.

```text
Input 5 WP → konfigurasi pada Raspberry Pi
FC → GPS terbaru → hitung jarak ke 5 WP → jarak ≤20m
    → simpan event ke SQLite → UDP → receiver ground → simpan + ACK
```

## File yang dipakai

| Lokasi | Program | Fungsi |
|---|---|---|
| Raspberry Pi | `SENDER2/waypoints.py` | Input konfigurasi, pemantauan jarak, penyimpanan trigger, retry UDP. |
| Raspberry Pi | `SENDER2/telemetry.py` | Membaca MAVLink FC; posisi baru dipakai ketika fix valid. |
| Ground | `SENDER2/tools/waypoint_receiver.py` | Penerima UDP contoh, deduplikasi persisten dan ACK setelah penyimpanan. |
| Komputer pengujian | `SENDER2/tools/simulate_waypoints.py` | Simulasi lima titik dengan UDP localhost dan satu ACK sengaja hilang. |
| Pengujian | `SENDER2/tests/test_waypoints.py` | Batas jarak, freshness, restart, ACK, input, dan identitas misi. |

Salin seluruh folder SENDER2 ke Raspberry Pi dan ground, lalu jalankan perintah dari **folder induk SENDER2**. Modul lain tidak otomatis berjalan.

## 1. Instalasi

Pada Raspberry Pi, gunakan virtual environment lalu:

```bash
python -m pip install -r SENDER2/requirements-onboard.txt
```

Pada ground, gunakan dependency utama (`requirements.txt`). Pembacaan FC memerlukan `pymavlink`; mode simulasi tidak memerlukannya.

## 2. Masukkan lima waypoint

Pada Raspberry Pi atau komputer yang menyiapkan konfigurasi:

```bash
python -m SENDER2.waypoints configure --mission misi_wp_001 --ground-host 192.168.1.100 --ground-port 5700
```

**Ganti alamat ground dengan alamat sebenarnya yang dapat dijangkau Pi.** Program meminta lima baris input secara bergantian:

```text
WP1 latitude longitude: -7.770000 110.370000
WP2 latitude longitude: ...
WP3 latitude longitude: ...
WP4 latitude longitude: ...
WP5 latitude longitude: ...
```

Angka WP1 di atas hanya contoh format, bukan target misi yang direkomendasikan. Gunakan titik desimal (`.`), urutan **latitude lalu longitude**, dan pertahankan tanda negatif. Format pasangan dipisahkan spasi atau koma. Nilai di luar rentang, nonfinite, dan koordinat identik ditolak.

Hasilnya `SENDER2/waypoints.config.json`. Jika konfigurasi sudah ada, pilih nama baru dengan `--output SENDER2/waypoints.misi002.json`; program tidak menimpa file lama. Bila dibuat di komputer lain, salin JSON tersebut ke Pi. JSON juga dapat diedit langsung sebelum misi.

Gunakan **mission_id baru untuk misi baru atau perubahan koordinat**, agar status waypoint misi sebelumnya tidak terpakai kembali. Konfigurasi lama dengan mission_id sama dan koordinat berubah akan ditolak saat membuka database.

## 3. Jalankan receiver pada ground

```powershell
.\.venv\Scripts\python.exe -m SENDER2.tools.waypoint_receiver --host 0.0.0.0 --port 5700
```

Receiver menampilkan ID waypoint, koordinat, serta apakah paket merupakan pengiriman ulang. Event tersimpan di `SENDER2/runs/waypoints/ground.sqlite3`, tabel `received`. Port 5700 terpisah dari port gambar UAV1 5600. Gunakan satu instance receiver per database/port.

Receiver default menolak paket bertanda simulasi. Gunakan `--allow-simulation` hanya saat pengujian. Simulasi otomatis pada langkah 5 sudah membuat receiver lokalnya sendiri.

## 4. Jalankan pemantau di Raspberry Pi

```bash
python -m SENDER2.waypoints run --config SENDER2/waypoints.config.json --fc /dev/ttyAMA0 --baud 57600
```

Sesuaikan port dan baud dengan FC sebenarnya. Program menampilkan jarak ke semua WP dan pesan `TRIGGER` ketika titik baru terpicu. Status ditulis ke `SENDER2/runs/waypoints/status.json`; database onboard adalah `SENDER2/runs/waypoints/state.sqlite3`.

Jika sender gambar juga membaca serial FC, **jangan membuka port serial yang sama dari dua proses**. Gunakan satu sumber telemetry yang dibagikan, misalnya endpoint terpisah dari MAVLink router. Opsi `--fc` menerima string koneksi pymavlink; contoh endpoint lokal yang sudah disediakan router: `udpin:127.0.0.1:14551`. Konfigurasi router tidak dibuat oleh modul ini.

Hentikan dengan Ctrl+C. Event yang belum mendapat ACK tetap tersimpan dan dicoba ulang saat program dijalankan kembali dengan mission_id/database yang sama. Jangan menghapus database saat resume misi.

## Aturan trigger dan pengiriman

- Jarak dihitung horizontal dari latitude/longitude menggunakan Haversine; altitude tidak digunakan. Nilai berasal dari estimasi posisi GPS, bukan jaminan jarak fisik bebas error.
- Sampel tanpa fix 3D atau lebih baik tidak dipakai; posisi lebih tua dari 2 detik ditolak secara default. Tidak ada GPS sintetis pada mode live.
- Pemeriksaan berdasarkan sampel posisi yang benar-benar diterima. Bila pesawat melintasi radius di antara dua sampel, trigger bisa terlewat; tidak ada interpolasi lintasan. Usahakan laju telemetry cukup untuk kecepatan fixed-wing dan ukur latency sebenarnya.
- Posisi yang berada tepat di target juga memicu. Tidak perlu pernah berada di luar radius sebelumnya.
- Setiap WP hanya menghasilkan satu **event logis per misi**. Paket UDP boleh dikirim ulang ketika ACK hilang. Ini pengiriman *at least once* dengan deduplikasi pada ground, bukan jaminan satu paket saja.
- Retry memakai event_id yang sama, timeout ACK 1 detik per percobaan, dan jeda retry bertahap sampai maksimum 30 detik. Pending tetap dikirim meskipun pesawat sudah keluar radius, karena event mencatat saat pertama masuk radius.
- `waiting` = belum terpicu; `pending` = terpicu namun belum mendapat ACK valid; `delivered` = ground telah mengonfirmasi penerimaan. `delivered` **tidak berarti copter sudah mendatangi/validasi titik**.
- Gunakan satu proses pemantau untuk UAV1/database yang sama. Titik yang sangat berdekatan boleh terpicu bersamaan.
- UDP memakai alamat ground yang dapat dijangkau dan ACK kembali ke alamat/port sumber. Port forwarding/VPN/firewall untuk internet harus disiapkan sesuai jaringan; uji localhost bukan validasi internet. Format ini belum memiliki autentikasi/enkripsi, sehingga gunakan jaringan tepercaya atau tunnel yang sesuai.

## Format UDP untuk integrasi base station

Satu event berupa JSON UTF-8, maksimum 1200 byte, dengan field:

```json
{
  "schema_version": 1,
  "type": "waypoint_trigger",
  "event_id": "UUID stabil untuk misi/UAV/waypoint",
  "mission_id": "misi_wp_001",
  "uav_id": "UAV1",
  "waypoint": {"waypoint_id": "WP1", "latitude_deg": -7.77, "longitude_deg": 110.37},
  "distance_m": 19.0,
  "radius_m": 20,
  "triggered_at_unix_s": 1793232000.0,
  "position": {
    "latitude_deg": -7.77017,
    "longitude_deg": 110.37,
    "position_timestamp_unix_s": 1793232000.0,
    "fix_type": 3,
    "simulation": false
  },
  "simulation": false
}
```

Contoh di atas menjelaskan format; nilai koordinat/timestamp/event_id bukan paket uji siap kirim. **Koordinat target copter ada di `waypoint`**, sedangkan `position` adalah posisi fixed-wing saat trigger.

Sesudah menyimpan event secara persisten dan melakukan deduplikasi berdasarkan event_id, ground membalas ke alamat pengirim:

```json
{"type":"waypoint_ack","event_id":"UUID yang sama dengan paket masuk","accepted":true}
```

ACK salah event_id atau tanpa `accepted: true` tidak menandai titik sebagai delivered. Jika memakai receiver base station yang sudah ada, tambahkan kontrak ACK/deduplikasi ini; penerima UDP yang hanya mencetak pesan akan membuat sender terus mencoba ulang.

## 5. Simulasi tanpa wahana

```powershell
.\.venv\Scripts\python.exe -m SENDER2.tools.simulate_waypoints
```

Simulator membuat lima koordinat sintetis, menggerakkan posisi uji ke jarak 35 → 19 → 10 → 35 meter dari setiap titik, dan mengirim melalui UDP localhost. Ini menjalankan inti trigger dan receiver, tidak menguji sensor/FC.

Hasil uji 29 Oktober 2026:

- 35 m: tidak memicu; 19 m: satu event; 10 m dan kembali ke 35 m: tidak membuat event kedua.
- Lima waypoint unik diterima ground, semuanya mendapat ACK.
- Satu ACK sengaja dihilangkan: sender melakukan retry, total 6 datagram, ground tetap menyimpan 5 event unik.
- Laporan run: `SENDER2/runs/waypoint_simulation/32d36b040f12/report.json`.
- Delapan kasus tes unit waypoint lulus; keseluruhan suite SENDER2: **44 passed, 1 warning** (deprecation TestClient). Mode FC live belum diuji pada Raspberry Pi/flight controller asli.
