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

Assert-Contains $PidSource 's_pd_err0 = err;' 'PD input must keep the Err direction'
Assert-NotContains $PidSource 's_pd_err0 = -err;' 'PD still reverses the Err direction'

function Get-FirstPdAngle([float]$Err) {
    $Out = 150.0 + 0.85 * $Err + 1.15 * $Err
    return [Math]::Max(100.0, [Math]::Min(175.0, $Out))
}

$Negative = Get-FirstPdAngle -Err (-10.0)
$Positive = Get-FirstPdAngle -Err 10.0
if ($Negative -ge 150.0 -or $Positive -le 150.0) {
    throw 'Positive and negative Err do not steer to the expected sides'
}
if ($Negative -lt 100.0 -or $Positive -gt 175.0) {
    throw 'Steering direction model exceeds servo limits'
}

Write-Output 'PASS steering direction'
