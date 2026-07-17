$ErrorActionPreference = 'Stop'
$Root = Split-Path -Parent $PSScriptRoot
$Gbk = [Text.Encoding]::GetEncoding(936)

function Assert-Contains([string]$Text, [string]$Expected, [string]$Message) {
    if (-not $Text.Contains($Expected)) { throw $Message }
}

$CameraPath = Join-Path $Root 'Seekfree_TC264_Opensource_Library/code/Camera.c'
$Camera = [IO.File]::ReadAllText($CameraPath, $Gbk)

Assert-Contains $Camera '#include "Shared.h"' 'Camera display does not import the shared Err value'
Assert-Contains $Camera 'ips200_show_string(120, 225, "Err:");' 'Err label is not placed in the bottom bar'
Assert-Contains $Camera 'ips200_show_float(152, 225, Err, 3, 2);' 'Err value is not shown with the required bottom-bar format'

Write-Output 'PASS camera Err display'
