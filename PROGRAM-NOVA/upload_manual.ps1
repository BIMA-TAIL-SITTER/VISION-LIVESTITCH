#!/usr/bin/env pwsh
# =============================================================================
# PROGRAM-NOVA | upload_manual.ps1
# Upload program ke bimaswarm-4 satu per satu dengan SCP
# Jalankan dari folder PROGRAM-NOVA:
#   cd "d:\KULIAH\BIMA - SWARM\VISION-LIVESTITCH\PROGRAM-NOVA"
#   .\upload_manual.ps1
# =============================================================================

$RASPI    = "bimaswarm-4@100.76.49.111"
$REMOTE   = "/home/bimaswarm-4/PROGRAM-NOVA"
$NOVA_DIR = $PSScriptRoot

$FILES = @(
    "config.py",
    "photo_capture.py",
    "exif_injector.py",
    "sender.py",
    "imu_monitor.py",
    "od_simulator.py",
    "requirements_raspi.txt"
)

Write-Host ""
Write-Host "===========================================================" -ForegroundColor Cyan
Write-Host " Upload PROGRAM-NOVA ke bimaswarm-4 (100.76.49.111)" -ForegroundColor Cyan
Write-Host "===========================================================" -ForegroundColor Cyan
Write-Host " CATATAN: Masukkan password saat diminta" -ForegroundColor Yellow
Write-Host "          Password akan diminta tiap file (normal)" -ForegroundColor Yellow
Write-Host "===========================================================" -ForegroundColor Cyan
Write-Host ""

# Buat folder remote (akan minta password 1x)
Write-Host "[1] Membuat folder ~/PROGRAM-NOVA di Raspi ..."
ssh $RASPI "mkdir -p $REMOTE"

Write-Host ""
Write-Host "[2] Mengupload $($FILES.Count) file ..." -ForegroundColor Cyan

$i = 0
foreach ($f in $FILES) {
    $i++
    $local = Join-Path $NOVA_DIR $f
    if (-not (Test-Path $local)) {
        Write-Host "    [$i/$($FILES.Count)] SKIP $f (tidak ada lokal)" -ForegroundColor Yellow
        continue
    }
    $size = [math]::Round((Get-Item $local).Length / 1KB, 1)
    Write-Host "    [$i/$($FILES.Count)] $f ($size KB) ..." -NoNewline
    scp -q "$local" "${RASPI}:${REMOTE}/$f"
    if ($LASTEXITCODE -eq 0) {
        Write-Host " ✓" -ForegroundColor Green
    } else {
        Write-Host " ✗ GAGAL" -ForegroundColor Red
    }
}

Write-Host ""
Write-Host "[3] Verifikasi file di Raspi ..."
ssh $RASPI "ls -lh $REMOTE/"

Write-Host ""
Write-Host "===========================================================" -ForegroundColor Green
Write-Host " Upload selesai!" -ForegroundColor Green
Write-Host "===========================================================" -ForegroundColor Green
Write-Host ""
Write-Host "Langkah selanjutnya – SSH ke Raspi dan install:" -ForegroundColor Yellow
Write-Host "  ssh bimaswarm-4@100.76.49.111" -ForegroundColor White
Write-Host "  cd ~/PROGRAM-NOVA" -ForegroundColor White
Write-Host "  pip3 install -r requirements_raspi.txt" -ForegroundColor White
Write-Host ""
Write-Host "Jalankan program:" -ForegroundColor Yellow
Write-Host "  python3 photo_capture.py --no-fc --host 100.95.70.29" -ForegroundColor White
Write-Host ""
