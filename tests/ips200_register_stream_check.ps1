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

Assert-Contains $Header 'void IPS200_ShowGrayImageFast(const uint8 *image, uint16 width, uint16 height);' 'Fast display API is missing'
Assert-Contains $Ips 'MODULE_QSPI2.BACONENTRY.U' 'Fast display does not write the QSPI2 BACON entry register'
Assert-Contains $Ips 'MODULE_QSPI2.DATAENTRY[0].U' 'Fast display does not feed the QSPI2 transmit FIFO directly'
Assert-Contains $Ips 'stream_config.B.DL = 31;' 'Fast display does not pack two RGB565 pixels per FIFO entry'
Assert-Contains $Ips '((uint32)ips200_gray_rgb565[image[0]] << 16)' 'Fast display does not place the first pixel in the high half-word'
Assert-Contains $Ips '| ips200_gray_rgb565[image[1]];' 'Fast display does not place the second pixel in the low half-word'
Assert-Contains $Ips 'width > IPS200_SCREEN_WIDTH || height > IPS200_SCREEN_HEIGHT' 'Fast display does not reject regions outside the screen'
Assert-Contains $Ips 'MODULE_QSPI2.STATUS.B.TXFIFOLEVEL >= IPS200_QSPI_FIFO_DEPTH' 'Fast display does not wait only when the FIFO is full'
Assert-Contains $Ips 'MODULE_QSPI2.STATUS.B.TXFIFOLEVEL > (IPS200_QSPI_FIFO_DEPTH - 2U)' 'Fast display does not reserve FIFO space for the final BACON entry'
Assert-Contains $Ips 'MODULE_QSPI2.GLOBALCON.B.RESETS = 7U;' 'Fast display timeout does not reset the QSPI state machine and FIFOs'
Assert-Contains $Cpu0 'IPS200_ShowGrayImageFast(mt9v03x_image[0], MT9V03X_W, MT9V03X_H);' 'CPU0 does not use the direct-register display path'
Assert-NotContains $Cpu0 'Camera_ShowDebug();' 'CPU0 still enters the slow library display path'

Write-Output 'PASS IPS200 register stream contract'
