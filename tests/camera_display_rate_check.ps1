$ErrorActionPreference = 'Stop'
$Root = Split-Path -Parent $PSScriptRoot
$Gbk = [Text.Encoding]::GetEncoding(936)

$Cpu0Path = Join-Path $Root 'Seekfree_TC264_Opensource_Library/user/cpu0_main.c'
$Cpu0 = [IO.File]::ReadAllText($Cpu0Path, $Gbk)

if (-not $Cpu0.Contains('#define DISPLAY_DEBUG_INTERVAL (10U)')) {
    throw 'CPU0 display debug refresh is not throttled to one frame in ten'
}

$FastDisplayPattern = '(?s)if\s*\(\+\+display_frame_count\s*>=\s*DISPLAY_DEBUG_INTERVAL\)\s*\{\s*display_frame_count\s*=\s*0;\s*Camera_ShowDebug\(\);\s*\}\s*else\s*\{\s*ips200_show_gray_image\(0,\s*0,\s*mt9v03x_image\[0\],\s*MT9V03X_W,\s*MT9V03X_H,\s*MT9V03X_W,\s*MT9V03X_H,\s*0\);\s*\}'
if ($Cpu0 -notmatch $FastDisplayPattern) {
    throw 'CPU0 does not use one bulk gray-image transfer on normal frames'
}

Write-Output 'PASS camera display rate policy'
