$ErrorActionPreference = 'Stop'
$Root = Split-Path -Parent $PSScriptRoot
$Gbk = [Text.Encoding]::GetEncoding(936)

function Read-Gbk([string]$RelativePath) {
    return [IO.File]::ReadAllText((Join-Path $Root $RelativePath), $Gbk)
}

function Assert-Contains([string]$Text, [string]$Expected, [string]$Message) {
    if (-not $Text.Contains($Expected)) { throw $Message }
}

function Assert-NotContains([string]$Text, [string]$Expected, [string]$Message) {
    if ($Text.Contains($Expected)) { throw $Message }
}

$Ips = Read-Gbk 'Seekfree_TC264_Opensource_Library/code/IPS200.c'
$Header = Read-Gbk 'Seekfree_TC264_Opensource_Library/code/IPS200.h'
$Cpu0 = Read-Gbk 'Seekfree_TC264_Opensource_Library/user/cpu0_main.c'
$DeviceHeader = Read-Gbk 'Seekfree_TC264_Opensource_Library/libraries/zf_device/zf_device_ips200.h'

Assert-Contains $Header 'void IPS200_ShowGrayImageFast(const uint8 *image, uint16 width, uint16 height);' 'Fast display API is missing'
Assert-Contains $DeviceHeader '#define IPS200_USE_SOFT_SPI             (1 )' 'IPS200 is no longer configured for the current software-SPI wiring'
Assert-Contains $DeviceHeader '#define IPS200_SCL_PIN                  (P15_4)' 'Unexpected IPS200 SCL wiring'
Assert-Contains $DeviceHeader '#define IPS200_SDA_PIN                  (P15_2)' 'Unexpected IPS200 SDA wiring'
Assert-Contains $Ips '#include "IfxPort_reg.h"' 'Fast display does not include the TC264 port register definition'
Assert-Contains $Ips 'MODULE_P15.OMR.U' 'Fast display does not write the P15 output register directly'
Assert-Contains $Ips 'IPS200_WriteBitsDirect' 'Fast display direct-register serializer is missing'
Assert-Contains $Ips 'IPS200_WriteCommandDirect(0x2AU);' 'Fast display does not program the column window directly'
Assert-Contains $Ips 'IPS200_WriteCommandDirect(0x2BU);' 'Fast display does not program the row window directly'
Assert-Contains $Ips 'IPS200_WriteCommandDirect(0x2CU);' 'Fast display does not start direct pixel streaming'
Assert-Contains $Ips 'IPS200_WriteBitsDirect(ips200_gray_rgb565[*image], 16U);' 'Fast display does not stream RGB565 pixels directly'
Assert-Contains $Ips 'width > IPS200_SCREEN_WIDTH || height > IPS200_SCREEN_HEIGHT' 'Fast display does not reject regions outside the screen'
Assert-NotContains $Ips 'IPS200_SPI' 'Fast display still references a macro hidden by IPS200_USE_SOFT_SPI'
Assert-NotContains $Ips 'MODULE_QSPI2' 'Fast display still drives QSPI2 pins instead of the configured screen pins'
Assert-NotContains $Ips 'spi_write_' 'Fast display still calls the unconfigured hardware-SPI driver'
Assert-Contains $Cpu0 'IPS200_ShowGrayImageFast(mt9v03x_image[0], MT9V03X_W, MT9V03X_H);' 'CPU0 does not use the direct-register display path'
Assert-NotContains $Cpu0 'Camera_ShowDebug();' 'CPU0 still enters the slow library display path'

Write-Output 'PASS IPS200 register stream contract'
