$ErrorActionPreference = 'Stop'
$Root = Split-Path -Parent $PSScriptRoot
$Gbk = [Text.Encoding]::GetEncoding(936)

$Cpu0Path = Join-Path $Root 'Seekfree_TC264_Opensource_Library/user/cpu0_main.c'
$Cpu0 = [IO.File]::ReadAllText($Cpu0Path, $Gbk)
$Cpu1Path = Join-Path $Root 'Seekfree_TC264_Opensource_Library/user/cpu1_main.c'
$Cpu1 = [IO.File]::ReadAllText($Cpu1Path, $Gbk)
$SharedPath = Join-Path $Root 'Seekfree_TC264_Opensource_Library/code/Shared.h'
$Shared = [IO.File]::ReadAllText($SharedPath, $Gbk)

if (-not $Cpu0.Contains('IPS200_ShowGrayImageFast(mt9v03x_image[0], MT9V03X_W, MT9V03X_H);')) {
    throw 'CPU0 does not use the direct-register display path for each frame'
}

if ($Cpu0.Contains('Camera_ShowDebug();')) {
    throw 'CPU0 still enters the slow full-debug display path'
}

if ($Cpu0.Contains('DISPLAY_DEBUG_INTERVAL')) {
    throw 'CPU0 still throttles between fast and slow display paths'
}

if (-not $Shared.Contains('extern volatile int16_t EncLeft;') -or
    -not $Shared.Contains('extern volatile int16_t EncRight;')) {
    throw 'Encoder display values are not shared between CPU1 and CPU0'
}

if (-not $Cpu1.Contains('volatile int16_t EncLeft  = 0;') -or
    -not $Cpu1.Contains('volatile int16_t EncRight = 0;') -or
    -not $Cpu1.Contains('int16_t  enc_left = 0, enc_right = 0;')) {
    throw 'CPU1 does not publish initialized encoder values'
}

if (-not $Cpu0.Contains('#define ENCODER_DISPLAY_DIV 5U') -or
    -not $Cpu0.Contains('if (++encoder_display_cnt >= ENCODER_DISPLAY_DIV)')) {
    throw 'Encoder display refresh is not limited to every five camera frames'
}

if (-not $Cpu0.Contains('ips200_show_string(2U, 128U, "L_Enc:");') -or
    -not $Cpu0.Contains('ips200_show_string(2U, 144U, "R_Enc:");') -or
    -not $Cpu0.Contains('ips200_show_int(58U, 128U, (int32)EncLeft, 5U);') -or
    -not $Cpu0.Contains('ips200_show_int(58U, 144U, (int32)EncRight, 5U);')) {
    throw 'IPS200 does not show both CPU1 encoder values below the camera image'
}

Write-Output 'PASS camera display rate policy'
