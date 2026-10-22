[CmdletBinding()]
param (
    [switch]$Quick,
    [switch]$Full,
    [int]$Runs = 0,
    [string]$OutputDir = "benchmarks/results"
)

$ErrorActionPreference = "Stop"

# Determine G++ executable location
$gpp = Get-Command "g++" -ErrorAction SilentlyContinue
if (-not $gpp) {
    $fallback = "C:\Program Files\CodeBlocks\MinGW\bin\g++.exe"
    if (Test-Path $fallback) {
        $gppPath = $fallback
        $env:PATH = "C:\Program Files\CodeBlocks\MinGW\bin;" + $env:PATH
    } else {
        Write-Error "Could not locate g++ in PATH or standard installation directory."
    }
} else {
    $gppPath = $gpp.Source
}

Write-Host "==========================================================" -ForegroundColor Cyan
Write-Host " Compiling BVH Benchmark Runner with $gppPath" -ForegroundColor Cyan
Write-Host "==========================================================" -ForegroundColor Cyan

$srcFiles = @(
    "benchmarks/src/benchmark_runner.cpp",
    "src/collision.cpp",
    "src/physics.cpp"
)

$outExe = "benchmarks/benchmark_runner.exe"
$includeFlags = @("-Iinclude", "-Isrc", "-Ibenchmarks/include")
$compileFlags = @("-std=c++20", "-O3", "-Wall", "-static-libgcc", "-static-libstdc++")

$allArgs = $compileFlags + $includeFlags + $srcFiles + @("-o", $outExe)

& $gppPath $allArgs
if ($LASTEXITCODE -ne 0) {
    Write-Error "Compilation failed with code $LASTEXITCODE"
}

Write-Host "`nCompilation successful! Binary: $outExe" -ForegroundColor Green
Write-Host "Starting benchmark execution...`n" -ForegroundColor Cyan

$runArgs = @("--output", $OutputDir)
if ($Quick) {
    $runArgs += "--quick"
} elseif ($Full) {
    $runArgs += "--full"
}
if ($Runs -gt 0) {
    $runArgs += @("--runs", $Runs)
}

& $outExe $runArgs
if ($LASTEXITCODE -ne 0) {
    Write-Error "Benchmark run failed with code $LASTEXITCODE"
}

Write-Host "Benchmark completed successfully." -ForegroundColor Green
