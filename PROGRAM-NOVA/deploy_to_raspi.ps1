#!/usr/bin/env pwsh
# =============================================================================
# PROGRAM-NOVA | deploy_to_raspi.ps1
# Upload program ke Raspberry Pi bimaswarm-4 via Tailscale
#
# Cara Jalankan di PowerShell:
#   .\deploy_to_raspi.ps1
#   .\deploy_to_raspi.ps1 -RaspiUser pi -RaspiIP 100.76.49.111
# =============================================================================

param(
    [string]$RaspiIP   = "100.76.49.111",        # IP Tailscale bimaswarm-4
    [string]$RaspiUser = "bimaswarm-4",          # Username Raspi
    [string]$RemoteDir = "/home/bimaswarm-4/PROGRAM-NOVA",
    [switch]$SkipInstall                    # Skip pip install
)

$ErrorActionPreference = "Stop"

# File-file yang di-upload ke Raspi (yang jalan di UAV saja)
$RASPI_FILES = @(
    "sender.py",
    "photo_capture.py",
    "exif_injector.py",
    "imu_monitor.py",
    "od_simulator.py",
    "config.py",
    "requirements_raspi.txt"
)

$LOCAL_DIR  = Split-Path $PSScriptRoot -Parent
$NOVA_DIR   = $PSScriptRoot
$RASPI_HOST = "$RaspiUser@$RaspiIP"

Write-Host ""
Write-Host "============================================================" -ForegroundColor Cyan
Write-Host " PROGRAM-NOVA Deploy to bimaswarm-4" -ForegroundColor Cyan
Write-Host "============================================================" -ForegroundColor Cyan
Write-Host "  Target    : $RASPI_HOST" -ForegroundColor Yellow
Write-Host "  Remote dir: $RemoteDir" -ForegroundColor Yellow
Write-Host "  Files     : $($RASPI_FILES.Count) file" -ForegroundColor Yellow
Write-Host ""

# ─── Cek apakah SSH tersedia ─────────────────────────────────────────────────
if (-not (Get-Command ssh -ErrorAction SilentlyContinue)) {
    Write-Host "ERROR: ssh tidak ditemukan. Install OpenSSH di Windows!" -ForegroundColor Red
    Write-Host "  Settings -> Optional Features -> OpenSSH Client"
    exit 1
}

if (-not (Get-Command scp -ErrorAction SilentlyContinue)) {
    Write-Host "ERROR: scp tidak ditemukan." -ForegroundColor Red
    exit 1
}

# ─── Test koneksi ────────────────────────────────────────────────────────────
Write-Host "[ 1/4 ] Testing koneksi SSH ke $RaspiIP ..." -ForegroundColor Cyan
try {
    $test = ssh -o ConnectTimeout=10 -o StrictHostKeyChecking=no `
                "$RASPI_HOST" "echo OK" 2>&1
    if ($test -ne "OK") {
        throw "SSH test gagal: $test"
    }
    Write-Host "        ✓ Koneksi OK!" -ForegroundColor Green
} catch {
    Write-Host "        ✗ Koneksi gagal: $_" -ForegroundColor Red
    Write-Host ""
    Write-Host "Troubleshooting:" -ForegroundColor Yellow
    Write-Host "  1. Pastikan Tailscale aktif di laptop ini"
    Write-Host "  2. Pastikan bimaswarm-4 Connected di Tailscale"
    Write-Host "  3. Coba: ssh $RASPI_HOST"
    Write-Host "  4. Jika gagal auth, buat SSH key:"
    Write-Host "     ssh-keygen -t ed25519 -C nova"
    Write-Host "     ssh-copy-id $RASPI_HOST"
    exit 1
}

# ─── Buat direktori remote ───────────────────────────────────────────────────
Write-Host ""
Write-Host "[ 2/4 ] Membuat direktori di Raspi ..." -ForegroundColor Cyan
ssh -o StrictHostKeyChecking=no "$RASPI_HOST" "mkdir -p $RemoteDir"
Write-Host "        ✓ $RemoteDir siap" -ForegroundColor Green

# ─── Upload file ─────────────────────────────────────────────────────────────
Write-Host ""
Write-Host "[ 3/4 ] Mengupload file ..." -ForegroundColor Cyan

$ok_count = 0
$fail_count = 0

foreach ($filename in $RASPI_FILES) {
    $local_path = Join-Path $NOVA_DIR $filename
    if (-not (Test-Path $local_path)) {
        Write-Host "        ! $filename tidak ditemukan (skip)" -ForegroundColor Yellow
        continue
    }

    Write-Host "        → $filename ..." -NoNewline
    try {
        scp -o StrictHostKeyChecking=no `
            "$local_path" `
            "${RASPI_HOST}:${RemoteDir}/${filename}" 2>&1 | Out-Null
        Write-Host " ✓" -ForegroundColor Green
        $ok_count++
    } catch {
        Write-Host " ✗ $_" -ForegroundColor Red
        $fail_count++
    }
}

Write-Host "        Upload selesai: $ok_count berhasil, $fail_count gagal" -ForegroundColor $(if ($fail_count -eq 0) {"Green"} else {"Yellow"})

# ─── Install dependencies di Raspi ──────────────────────────────────────────
if (-not $SkipInstall) {
    Write-Host ""
    Write-Host "[ 4/4 ] Install Python dependencies di Raspi ..." -ForegroundColor Cyan
    Write-Host "        (ini bisa memakan waktu 2-5 menit)" -ForegroundColor Gray

    $install_cmd = @"
cd $RemoteDir && \
echo '--- Updating pip ---' && \
pip3 install --upgrade pip --quiet && \
echo '--- Installing NOVA requirements ---' && \
pip3 install -r requirements_raspi.txt 2>&1 | tail -5 && \
echo 'INSTALL_OK'
"@

    $result = ssh -o StrictHostKeyChecking=no "$RASPI_HOST" "$install_cmd" 2>&1
    if ($result -match "INSTALL_OK") {
        Write-Host "        ✓ Dependencies terinstall!" -ForegroundColor Green
    } else {
        Write-Host "        ⚠ Install mungkin ada error, cek manual:" -ForegroundColor Yellow
        Write-Host "          ssh $RASPI_HOST 'cd $RemoteDir && pip3 install -r requirements_raspi.txt'"
    }
} else {
    Write-Host ""
    Write-Host "[ 4/4 ] Skip install (--SkipInstall)" -ForegroundColor Gray
}

# ─── Verifikasi ──────────────────────────────────────────────────────────────
Write-Host ""
Write-Host "============================================================" -ForegroundColor Cyan
Write-Host " DEPLOY SELESAI" -ForegroundColor Green
Write-Host "============================================================" -ForegroundColor Cyan
Write-Host ""
Write-Host "File di Raspi:" -ForegroundColor Yellow
ssh -o StrictHostKeyChecking=no "$RASPI_HOST" "ls -lh $RemoteDir/*.py" 2>&1

Write-Host ""
Write-Host "╔══════════════════════════════════════════════════════════╗" -ForegroundColor Cyan
Write-Host "║  CARA MENJALANKAN DI RASPI (bimaswarm-4)                ║" -ForegroundColor Cyan
Write-Host "╠══════════════════════════════════════════════════════════╣" -ForegroundColor Cyan
Write-Host "║                                                          ║"
Write-Host "║  SSH ke Raspi:                                           ║"
Write-Host "║    ssh $RASPI_HOST" -NoNewline
Write-Host "                           ║"
Write-Host "║                                                          ║"
Write-Host "║  Mode FOTO 1fps + EXIF GPS (DIREKOMENDASIKAN):          ║"
Write-Host "║    python3 ~/PROGRAM-NOVA/photo_capture.py               ║"
Write-Host "║                                                          ║"
Write-Host "║  Mode tanpa FC (testing kamera saja):                   ║"
Write-Host "║    python3 ~/PROGRAM-NOVA/photo_capture.py --no-fc      ║"
Write-Host "║                                                          ║"
Write-Host "║  EXIF Injector standalone (watch folder):               ║"
Write-Host "║    python3 ~/PROGRAM-NOVA/exif_injector.py \            ║"
Write-Host "║      --watch-dir /tmp/nova_photos                        ║"
Write-Host "║                                                          ║"
Write-Host "║  OD Simulator (kirim ke ground):                        ║"
Write-Host "║    python3 ~/PROGRAM-NOVA/od_simulator.py \             ║"
Write-Host "║      --host <IP_GROUND>                                  ║"
Write-Host "║                                                          ║"
Write-Host "╚══════════════════════════════════════════════════════════╝" -ForegroundColor Cyan
Write-Host ""
