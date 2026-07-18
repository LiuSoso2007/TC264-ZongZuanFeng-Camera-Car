$ErrorActionPreference = 'Stop'
$Root = Split-Path -Parent $PSScriptRoot
$Gbk = [Text.Encoding]::GetEncoding(936)

$Cpu0Path = Join-Path $Root 'Seekfree_TC264_Opensource_Library/user/cpu0_main.c'
$Cpu0 = [IO.File]::ReadAllText($Cpu0Path, $Gbk)

if (-not $Cpu0.Contains('IPS200_ShowGrayImageFast(mt9v03x_image[0], MT9V03X_W, MT9V03X_H);')) {
    throw 'CPU0 does not use the direct-register display path for each frame'
}

if ($Cpu0.Contains('Camera_ShowDebug();')) {
    throw 'CPU0 still enters the slow full-debug display path'
}

if ($Cpu0.Contains('DISPLAY_DEBUG_INTERVAL')) {
    throw 'CPU0 still throttles between fast and slow display paths'
}

Write-Output 'PASS camera display rate policy'
