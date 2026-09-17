<#
monitor_upload.ps1
プロジェクトルートで実行する想定。Windows PowerShell 5.1 で安全に動作するように
ASCII ベースで記述しています。
#>

# logs ディレクトリを安全に作成
$dir = '.\logs'
if (-not (Test-Path $dir -PathType Container)) {
    New-Item -ItemType Directory -Path $dir | Out-Null
}

# タイムスタンプ付きファイル名
# StreamWriter は .NET のカレントディレクトリ基準になるため絶対パスにする
$ts = Get-Date -Format 'yyyyMMddHHmmss'
$file = Join-Path (Resolve-Path $dir).Path ("serial_$ts.log")

# pio を探す（PATH -> 既知の場所 -> python -m platformio）
$pioCmd = $null
try { $cmd = Get-Command pio -ErrorAction Stop; $pioCmd = $cmd.Path } catch {}
if (-not $pioCmd) {
    $candidate1 = Join-Path $env:USERPROFILE '.platformio\penv\Scripts\pio.exe'
    if (Test-Path $candidate1) { $pioCmd = $candidate1 }
    else {
        $candidate2 = Join-Path $env:USERPROFILE 'AppData\Roaming\Python\Scripts\platformio.exe'
        if (Test-Path $candidate2) { $pioCmd = $candidate2 }
    }
}

if (-not $pioCmd) {
    Write-Host "pio not found in PATH; will try 'python -m platformio'" -ForegroundColor Yellow
    $usePython = $true
} else {
    Write-Host ("Using pio: " + $pioCmd) -ForegroundColor Green
    $usePython = $false
}

# upload 実行
if ($usePython) {
    python -m platformio run -t upload
    $rc = $LASTEXITCODE
} else {
    & $pioCmd run -t upload
    $rc = $LASTEXITCODE
}

if ($rc -ne 0) {
    Write-Host ("upload failed (exit " + $rc + ")") -ForegroundColor Red
    exit $rc
}

# monitor を実行してログへ追記（画面表示＋ファイル保存）
# pyserial (miniterm) を直接使って対話型の問題（UserSideException）を回避します。
Write-Host "Starting serial monitor (logging to $file)..." -ForegroundColor Green

$enc = New-Object System.Text.UTF8Encoding($false)   # $true にすると BOM 付き
$sw  = New-Object System.IO.StreamWriter($file, $true, $enc)

try {
    # platformio.ini からボーレートとポートを取得する
    $baud = 115200
    $port = $null
    if (Test-Path "platformio.ini") {
        $iniContent = Get-Content "platformio.ini" -Raw
        if ($iniContent -match 'monitor_speed\s*=\s*(\d+)') {
            $baud = [int]$Matches[1]
        }
        if ($iniContent -match 'monitor_port\s*=\s*([^\r\n]+)') {
            $port = $Matches[1].Trim()
        }
    }

    # プラットフォーム固有の python (PlatformIOの仮想環境内) またはシステムの python を探す
    $pyCmd = "python"
    $candidatePy = Join-Path $env:USERPROFILE '.platformio\penv\Scripts\python.exe'
    if (Test-Path $candidatePy) {
        $pyCmd = $candidatePy
    }

        # pyserial miniterm を実行 (--exit-char 3 を追加して Ctrl+C で終了できるようにする)
    if ($port) {
        & $pyCmd -m serial.tools.miniterm --exit-char 3 --eol LF $port $baud 2>&1 | ForEach-Object {
            $line = $_.ToString()
            Write-Host $line
            $sw.WriteLine($line)
            $sw.Flush()
        }
    } else {
        & $pyCmd -m serial.tools.miniterm --exit-char 3 --eol LF $baud 2>&1 | ForEach-Object {
            $line = $_.ToString()
            Write-Host $line
            $sw.WriteLine($line)
            $sw.Flush()
        }
    }
}
finally {
    $sw.Close()
}