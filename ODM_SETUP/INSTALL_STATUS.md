# Hasil instalasi — 2 November 2026

Setup berhasil dipasang dan dijalankan pada ground Windows ini.

| Komponen | Hasil |
|---|---|
| WSL2 | 2.6.3.0; Ubuntu-22.04 berhasil menjalankan Linux |
| Docker Desktop | 4.97.0, instalasi per pengguna, backend WSL2 |
| Docker Engine | 29.9.2, Linux containers |
| Docker Compose | 5.6.1 |
| NodeODM API | `/info` merespons; version 3.6.2 |
| ODM engine | 3.6.2, sesuai respons API |
| PyODM | 1.5.12, environment `ODM_SETUP/.venv` |
| UI | http://localhost:3000 mengembalikan HTTP 200 |
| Container | `odm_setup-nodeodm-1`, running saat verifikasi |
| Batas container | 8 GiB RAM, 4 CPU; diverifikasi lewat Docker inspect |
| Paralel tugas | 1; diverifikasi lewat API |
| Penyimpanan | bind mount `ODM_SETUP/data` di drive D: ke `/var/www/data` |

Image Linux/amd64 dikunci pada digest:
`sha256:dde9333fe1f89fc6164901439c58e0971a52a67ae3a5c955ae77643aa9d644f8`.
API melaporkan kapasitas host WSL, yang bisa lebih besar dari batas container.
Batas aktual container diperiksa terpisah melalui Docker inspect.

## Verifikasi yang sudah dilakukan

- PyODM berhasil mengambil informasi NodeODM dan daftar opsi pemrosesan.
- Opsi `orthophoto-resolution`, `skip-3dmodel`, `max-concurrency` tersedia.
- Konfigurasi Compose valid; container aktif; UI HTTP 200.
- Tiga pengujian helper lulus: deduplikasi tanpa mengubah sumber, pencatatan UUID
  dan penolakan submit ulang, input kosong tidak membuat catatan tugas.
- `pip check`: tidak ada dependency yang rusak. Versi lengkap ada di `requirements.lock.txt`.
- Scan DATASET 4: 273 foto unik, 2 duplikat identik diabaikan untuk upload.

## Batas verifikasi

Belum menjalankan rekonstruksi dataset penuh, menghasilkan GeoTIFF, mengukur
akurasi peta, atau menguji restart ketika ada tugas aktif. Pengujian submit UUID
adalah unit test dengan NodeODM tiruan; koneksi `/info`, `/options`, dan UI diuji
ke container nyata. Status instalasi berhasil tidak berarti kualitas peta sudah
terverifikasi.

Setup memakai CPU. Integrasi kamera DJI, sinkronisasi otomatis arsip Raspberry Pi,
dan integrasi layer ke base station tidak ditambahkan pada pekerjaan instalasi ini.
Setelah Windows restart, nyalakan Docker Desktop lalu jalankan `compose up -d`
sesuai README. Tidak diperlukan restart Windows selama instalasi ini.
