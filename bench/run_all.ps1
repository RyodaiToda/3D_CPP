# C++ / Python 物理エンジン比較ベンチを一括実行する。
#
#   pwsh bench\run_all.ps1                 # 全部（数分〜十数分）
#   pwsh bench\run_all.ps1 -SkipSweep      # スケーリング測定を飛ばす
#   pwsh bench\run_all.ps1 -SkipGif        # GIF 生成を飛ばす
#
# 生成物は bench\results\ に出る。

[CmdletBinding()]
param(
    [int]$PileN = 256,
    [int]$PileSteps = 900,
    [switch]$SkipSweep,
    [switch]$SkipGif,
    [switch]$SkipPython,
    [string]$EngineRoot = ""
)

$ErrorActionPreference = "Stop"

# 日本語混じりの出力が CP932 で化けないようにする
$env:PYTHONIOENCODING = "utf-8"
[Console]::OutputEncoding = [System.Text.Encoding]::UTF8

$BenchDir = Split-Path -Parent $MyInvocation.MyCommand.Path
$Root = Split-Path -Parent $BenchDir
$Results = Join-Path $BenchDir "results"
New-Item -ItemType Directory -Force -Path $Results | Out-Null

function Find-CMake {
    $c = Get-Command cmake -ErrorAction SilentlyContinue
    if ($c) { return $c.Source }
    $candidates = @(
        "C:\Program Files\CMake\bin\cmake.exe",
        "C:\Program Files (x86)\Microsoft Visual Studio\18\BuildTools\Common7\IDE\CommonExtensions\Microsoft\CMake\CMake\bin\cmake.exe",
        "C:\Program Files\Microsoft Visual Studio\2022\Community\Common7\IDE\CommonExtensions\Microsoft\CMake\CMake\bin\cmake.exe"
    )
    foreach ($p in $candidates) { if (Test-Path $p) { return $p } }
    throw "cmake が見つかりません。PATH に通すか run_all.ps1 の候補リストに追加してください。"
}

# ------------------------------------------------------------------ ビルド
$cmake = Find-CMake
$BuildDir = Join-Path $Root "build"
Write-Host "==> building bench  ($cmake)" -ForegroundColor Cyan
& $cmake -S $Root -B $BuildDir | Out-Null
& $cmake --build $BuildDir --target bench --config Release | Out-Null
if ($LASTEXITCODE -ne 0) { throw "ビルドに失敗しました" }

$bench = Join-Path $BuildDir "Release\bench.exe"
if (-not (Test-Path $bench)) { $bench = Join-Path $BuildDir "bench.exe" }
if (-not (Test-Path $bench)) { throw "bench.exe が見つかりません: $BuildDir" }

$pyArgs = @()
if ($EngineRoot -ne "") { $pyArgs = @("--engine", $EngineRoot) }

# ------------------------------------------------------------ 本体の 4 シーン
Write-Host "==> main scenes" -ForegroundColor Cyan
foreach ($cfg in @("matched", "warmonly", "positiononly", "default")) {
    & $bench --out (Join-Path $Results "cpp_$cfg.json") --config $cfg `
             --pile-n $PileN --pile-steps $PileSteps
}
if (-not $SkipPython) {
    & python (Join-Path $BenchDir "bench_py.py") --out (Join-Path $Results "py.json") `
             --pile-n $PileN --pile-steps $PileSteps @pyArgs
}

# ------------------------------------------------------------- スケーリング
if (-not $SkipSweep) {
    Write-Host "==> scaling sweep" -ForegroundColor Cyan
    foreach ($n in @(64, 128, 256, 512, 1024, 2048)) {
        & $bench --out (Join-Path $Results "sweep_cpp_$n.json") --config matched `
                 --scene pile --pile-n $n --pile-steps 300
    }
    if (-not $SkipPython) {
        foreach ($n in @(16, 32, 64, 128, 256)) {
            & python (Join-Path $BenchDir "bench_py.py") `
                     --out (Join-Path $Results "sweep_py_$n.json") `
                     --scene pile --pile-n $n --pile-steps 300 @pyArgs
        }
    }
}

# --------------------------------------------------------------- レポート
Write-Host "==> report + charts" -ForegroundColor Cyan
$report = Join-Path $Results "report.txt"
& python (Join-Path $BenchDir "compare.py") `
    "python=$(Join-Path $Results 'py.json')" `
    "cpp-matched=$(Join-Path $Results 'cpp_matched.json')" `
    "cpp-warmonly=$(Join-Path $Results 'cpp_warmonly.json')" `
    "cpp-positiononly=$(Join-Path $Results 'cpp_positiononly.json')" `
    "cpp-default=$(Join-Path $Results 'cpp_default.json')" | Out-File -FilePath $report -Encoding utf8
Get-Content $report | Select-Object -First 30

& python (Join-Path $BenchDir "plot.py") --results $Results --out $Results

# --------------------------------------------------------------------- GIF
if (-not $SkipGif) {
    Write-Host "==> traces + gif" -ForegroundColor Cyan
    # トレース出力はタイマーを汚すので、計測とは別に走らせる
    foreach ($cfg in @("matched", "default")) {
        & $bench --out (Join-Path $Results "_trace_run.json") --config $cfg --scene stack `
                 --trace (Join-Path $Results "trace_cpp_${cfg}_stack.json") --trace-stride 10
    }
    & $bench --out (Join-Path $Results "_trace_run.json") --config matched --scene pile `
             --pile-n $PileN --pile-steps $PileSteps `
             --trace (Join-Path $Results "trace_cpp_matched_pile.json") --trace-stride 10
    if (-not $SkipPython) {
        & python (Join-Path $BenchDir "bench_py.py") --out (Join-Path $Results "_trace_run.json") `
                 --scene stack --trace (Join-Path $Results "trace_py_stack.json") `
                 --trace-stride 10 @pyArgs
        & python (Join-Path $BenchDir "bench_py.py") --out (Join-Path $Results "_trace_run.json") `
                 --scene pile --pile-n $PileN --pile-steps $PileSteps `
                 --trace (Join-Path $Results "trace_py_pile.json") --trace-stride 10 @pyArgs
    }
    Remove-Item (Join-Path $Results "_trace_run.json") -ErrorAction SilentlyContinue

    & python (Join-Path $BenchDir "gif_labels.py") --results $Results
}

Write-Host "==> done -> $Results" -ForegroundColor Green
