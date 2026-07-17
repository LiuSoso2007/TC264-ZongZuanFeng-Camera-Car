$ErrorActionPreference = 'Stop'
$Root = Split-Path -Parent $PSScriptRoot
$Gbk = [Text.Encoding]::GetEncoding(936)

function Read-Gbk([string]$RelativePath) {
    return [IO.File]::ReadAllText((Join-Path $Root $RelativePath), $Gbk)
}

function Assert-Contains([string]$Text, [string]$Expected, [string]$Message) {
    if (-not $Text.Contains($Expected)) { throw $Message }
}

function Assert-NotContains([string]$Text, [string]$Unexpected, [string]$Message) {
    if ($Text.Contains($Unexpected)) { throw $Message }
}

$PidSource = Read-Gbk 'Seekfree_TC264_Opensource_Library/code/PID.c'

Assert-Contains $PidSource 's_pd_err0 = Err;' 'PD input must keep the Err direction'
Assert-NotContains $PidSource 's_pd_err0 = -Err;' 'PD still reverses the Err direction'

function Get-FirstPdAngle([float]$Err) {
    $Out = 80.0 + 2.5 * $Err + 0.4 * $Err
    return [Math]::Max(9.0, [Math]::Min(132.0, $Out))
}

$Negative = Get-FirstPdAngle -Err (-10.0)
$Positive = Get-FirstPdAngle -Err 10.0
if ($Negative -ge 80.0 -or $Positive -le 80.0) {
    throw 'Positive and negative Err do not steer to the expected sides'
}
if ($Negative -lt 9.0 -or $Positive -gt 132.0) {
    throw 'Steering direction model exceeds servo limits'
}

Write-Output 'PASS steering direction'
