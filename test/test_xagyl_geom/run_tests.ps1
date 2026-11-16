# =============================================================================
#  run_tests.ps1 —— 几何库单元测试
#
#  用 PlatformIO 自带的 MinGW 工具链直接编译 "测试 + 库 + Unity" 并运行，
#  绕开 native 环境下 LDF 对本地 lib 的自动发现（该发现机制在此环境不稳定）。
#
#  用法：  powershell -ExecutionPolicy Bypass -File test\test_xagyl_geom\run_tests.ps1
#  退出码：0 = 全部通过；非 0 = 有失败。
# =============================================================================
$ErrorActionPreference = 'Stop'

$root = (Resolve-Path (Join-Path $PSScriptRoot '..\..')).Path
$libSrc = Join-Path $root 'lib\xagyl_geom\src'
$unitySrc = Join-Path $root '.pio\libdeps\native\Unity\src'
$buildDir = Join-Path $root '.pio\geom-test'

# 1. 找 g++：先 PlatformIO 自带 MinGW，再 PATH
$mingw = Join-Path $env:USERPROFILE '.platformio\packages\toolchain-gccmingw32\bin\g++.exe'
if (Test-Path -LiteralPath $mingw) {
    $gpp = $mingw
} else {
    $found = Get-Command g++ -ErrorAction SilentlyContinue
    if ($found) { $gpp = $found.Source } else { $gpp = $null }
}
if (-not $gpp) {
    Write-Host '找不到 g++。先执行: python -m platformio pkg install -g -t toolchain-gccmingw32'
    exit 2
}
$env:PATH = (Split-Path -Parent $gpp) + ';' + $env:PATH

# 2. 找 Unity 源码
if (-not (Test-Path -LiteralPath (Join-Path $unitySrc 'unity.c'))) {
    $alt = Get-ChildItem -LiteralPath (Join-Path $root '.pio') -Recurse -Include 'unity.c' -ErrorAction SilentlyContinue | Select-Object -First 1
    if ($alt) { $unitySrc = $alt.DirectoryName } else {
        Write-Host '未找到 Unity，先执行一次: python -m platformio run -e native'
        exit 2
    }
}

if (-not (Test-Path -LiteralPath $buildDir)) { New-Item -ItemType Directory -Path $buildDir | Out-Null }
$exe = Join-Path $buildDir 'geom_test.exe'
if (Test-Path -LiteralPath $exe) { Remove-Item -LiteralPath $exe -Force }

# 3. 编译（源文件顺序：测试、库、C 版 Unity）
Write-Host 'compiling...'
& $gpp -std=gnu++11 -Wall -Wextra "-I$unitySrc" "-I$libSrc" -o $exe `
    (Join-Path $PSScriptRoot 'test_xagyl_geom.cpp') `
    (Join-Path $libSrc 'xagyl_geom.cpp') `
    (Join-Path $unitySrc 'unity.c')
if ($LASTEXITCODE -ne 0) { Write-Host 'compile failed'; exit 1 }

# 4. 运行
Write-Host 'running...'
& $exe
$rc = $LASTEXITCODE
Remove-Item -LiteralPath $exe -Force -ErrorAction SilentlyContinue
if ($rc -eq 0) { Write-Host 'ALL TESTS PASSED' } else { Write-Host 'TESTS FAILED' }
exit $rc
