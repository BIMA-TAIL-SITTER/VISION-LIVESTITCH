# ODM di ground — Windows + Docker + PyODM

Setup ini terpisah dari SENDER2 live stitching. NodeODM menjalankan ODM untuk
rekonstruksi dari **foto asli setelah penerbangan**, kemudian menghasilkan
orthophoto. Jalankan satu tugas untuk UAV1 dan tugas lain untuk UAV2. Jangan
campurkan foto kedua UAV apabila hasilnya harus terpisah.

## 1. Komponen dan lokasi

- Raspberry Pi: capture, timestamp/telemetry, arsip foto asli, pengiriman preview SENDER2.
- Ground Windows: Docker Desktop (WSL2), container NodeODM, Python/PyODM.
- NodeODM: API dan UI sederhana di http://localhost:3000.
- PyODM: klien Python yang mengirim pekerjaan ke NodeODM; bukan mesin pemroses.
- WebODM adalah dashboard tambahan, **tidak diperlukan** untuk setup ini.
- Sinkronisasi arsip Pi ke ground dan integrasi base station belum otomatis.

Folder setup: `D:\KULIAH\BIMA - SWARM\VISION-LIVESTITCH\ODM_SETUP`.
Data NodeODM tersimpan di `data`, catatan UUID di `jobs`, hasil unduhan di
`results`. Folder tersebut dan `.venv` diabaikan Git.

## 2. Instalasi awal / pemasangan ulang

Pada komputer ini WSL2 Ubuntu sudah dapat berjalan. Di komputer baru, pasang
WSL2 sesuai panduan Microsoft/Docker dahulu dan restart jika diminta installer.
Docker Desktop memakai Linux containers. Jangan menjalankan dua instalasi
Docker Engine berbeda di WSL dan Docker Desktop untuk setup ini.

PowerShell:

```powershell
winget install --id Docker.DockerDesktop --exact --source winget --accept-source-agreements --accept-package-agreements --override 'install --user --quiet --accept-license --backend=wsl-2'
```

Buka terminal baru setelah instalasi agar PATH diperbarui. Alternatif bila
`docker` belum dikenali: gunakan executable lokal seperti langkah berikut.

Lingkungan Python khusus (sudah dibuat pada komputer ini):

```powershell
Set-Location 'D:\KULIAH\BIMA - SWARM\VISION-LIVESTITCH'
.\.venv\Scripts\python.exe -m venv ODM_SETUP\.venv
.\ODM_SETUP\.venv\Scripts\python.exe -m pip install -r ODM_SETUP\requirements.txt
```

Pada komputer lain tanpa `.venv` proyek, ganti perintah pembuatan dengan
`py -3 -m venv ODM_SETUP\.venv` setelah memasang Python.

## 3. Nyalakan layanan

```powershell
Set-Location 'D:\KULIAH\BIMA - SWARM\VISION-LIVESTITCH\ODM_SETUP'
$dockerExe = "$env:LOCALAPPDATA\Programs\DockerDesktop\resources\bin\docker.exe"
$env:PATH = (Split-Path $dockerExe) + ';' + $env:PATH
& $dockerExe desktop start
& $dockerExe info
& $dockerExe compose pull
& $dockerExe compose up -d
& $dockerExe compose ps
.\.venv\Scripts\python.exe odm_job.py check
```

`compose pull` hanya perlu pada pemasangan. Image Linux/amd64 dikunci dengan
digest di `compose.yaml`; pembaruan versi dilakukan secara sengaja dengan
mengganti digest sesudah memeriksa versi baru.
Unduhan image besar dan bisa memerlukan beberapa menit. Jika Docker dipasang
untuk semua pengguna, executable biasanya di
`C:\Program Files\Docker\Docker\resources\bin\docker.exe`.

Konfigurasi awal: RAM container maksimal 8 GB, 4 CPU, satu pekerjaan diproses
pada satu waktu. Ini titik awal untuk laptop RAM 24 GB, bukan jaminan cukup
untuk semua dataset. Docker/WSL juga memiliki batas RAM sendiri. UI hanya
dibuka di localhost; belum untuk akses komputer lain atau internet.

## 4. Siapkan foto per misi dan UAV

Contoh: `dataset\misi01\UAV1` dan `dataset\misi01\UAV2`.

1. Salin **arsip foto asli** dari setiap Pi ke ground. Jangan mengambil hasil
   mosaic live sebagai input ODM. Pastikan transfer selesai sebelum submit.
2. Pertahankan EXIF dan salinan asli. Helper hanya membaca foto, tidak mengubahnya.
3. Pilih foto tajam dengan overlap; hindari foto belokan tajam, langit, dan blur.
   Konsistensi mode kamera, fokus, dan focal length membantu rekonstruksi.
4. Periksa koordinat EXIF dan timestamp capture. Kehadiran GPS belum menjamin
   koordinatnya benar. Jika GPS berasal dari flight controller, cocokkan waktu
   capture dengan telemetry; jangan memakai waktu diterimanya gambar di ground.
5. Untuk lokasi dari luar EXIF, gunakan mekanisme `geo.txt` resmi ODM melalui
   pekerjaan yang dikonfigurasi untuk itu. Helper sederhana ini belum menerima
   `geo.txt` atau GCP. Jangan membuat nilai GPS/altitude palsu agar proses lolos.
6. Samakan satuan dan datum altitude. File kalibrasi OpenCV SENDER2 tidak langsung
   menjadi konfigurasi kamera ODM. Pada percobaan pertama biarkan ODM melakukan
   estimasi kamera, lalu evaluasi sebelum menerapkan kalibrasi khusus.

Untuk DATASET 4, terdapat 275 JPG dan 273 gambar unik berdasarkan pemeriksaan
sebelumnya. Helper membuang duplikat **berdasarkan SHA256 isi file** saat upload;
file asli tidak dihapus. Helper membaca foto langsung dalam folder, tidak rekursif.
Dataset harus diidentifikasi berasal dari UAV yang mana sebelum dipakai sebagai
hasil misi operasional.

## 5. Kirim pekerjaan

Semua perintah berikut dijalankan dari folder `ODM_SETUP`:

```powershell
.\.venv\Scripts\python.exe odm_job.py submit --images '..\dataset\DATASET 4' --name dataset4-UAV1 --resolution 5
```

Ini membuat pekerjaan baru dan menyimpan `jobs\dataset4-UAV1.json` berisi UUID,
daftar foto, duplikat, serta opsi. Nama yang sudah ada ditolak untuk menghindari
submit berulang tanpa sengaja. Jika upload terputus sebelum UUID tercatat,
periksa daftar tugas pada UI NodeODM sebelum mencoba lagi; tugas mungkin sudah
terbuat. Jangan menghapus catatan dan mengulang submit secara membabi buta.

Opsi awal: `orthophoto-resolution=5` (cm/pixel), `skip-3dmodel=true`,
`max-concurrency=4`. Resolusi 5 cm/pixel **bukan** akurasi posisi 5 cm dan tidak
menambah detail nyata jika GSD foto lebih kasar. Model mesh 3D bertekstur dilewati;
tahap rekonstruksi yang diperlukan orthophoto tetap berjalan.

## 6. Pantau dan unduh

```powershell
.\.venv\Scripts\python.exe odm_job.py status --job jobs\dataset4-UAV1.json
.\.venv\Scripts\python.exe odm_job.py download --job jobs\dataset4-UAV1.json --wait
```

`--wait` menunggu tugas yang sama, tidak membuat tugas baru. Ctrl+C menghentikan
klien yang menunggu; pekerjaan dalam NodeODM tetap berjalan. Jalankan perintah
download kembali untuk melanjutkan pemantauan. Hasil diekstrak ke `results\UUID`.
Orthophoto umumnya berada di `odm_orthophoto\odm_orthophoto.tif` dalam hasil itu.

Untuk UAV2, gunakan folder foto UAV2 dan `--name misi01-UAV2`. Dua tugas boleh
masuk antrean, tetapi konfigurasi ini memprosesnya bergantian untuk membatasi RAM.

## 7. Validasi hasil

Buka GeoTIFF di QGIS. Periksa CRS/lokasi, kelengkapan area, seam, objek ganda,
distorsi jalan/bangunan, dan laporan rekonstruksi bila tersedia. Bandingkan
dengan titik cek independen untuk menilai akurasi; waypoint misi bukan otomatis
GCP. Tanpa georeferensi yang benar, hasil visual belum layak menjadi peta posisi.
Hasil ODM juga perlu evaluasi walaupun proses berstatus COMPLETED.

## 8. Penyimpanan, log, dan berhenti

```powershell
& $dockerExe compose logs --tail 100 nodeodm
& $dockerExe compose stop
# Jalankan lagi saat diperlukan:
& $dockerExe compose up -d
```

Selesaikan pekerjaan sebelum menghentikan container. `stop` mempertahankan data.
NodeODM dikonfigurasi membersihkan tugas lama setelah 43.200 menit (30 hari).
Unduh dan cadangkan hasil serta foto asli; folder `results` tidak berada di dalam
penyimpanan NodeODM sehingga tidak ikut pembersihan tugas tersebut. Jangan
menghapus `data` saat masih membutuhkan tugas. Image Docker/virtual disk WSL
tetap memakai drive sistem; bind mount hasil NodeODM di folder proyek memakai D:.

Jika port 3000 sudah digunakan, identifikasi layanan pemakainya sebelum mengubah
port. Jika API gagal, cek `compose ps` dan log. Jika pemrosesan kehabisan RAM,
mulai dari subset foto yang saling overlap atau sesuaikan kualitas/kapasitas;
jangan menganggap semua dataset bisa selesai dengan batas 8 GB.

## Referensi resmi

- https://docs.docker.com/desktop/setup/install/windows-install/
- https://github.com/OpenDroneMap/NodeODM
- https://github.com/OpenDroneMap/PyODM
- https://docs.opendronemap.org/
