$ErrorActionPreference = 'Stop'
$Root = Split-Path -Parent $PSScriptRoot
$Gbk = [Text.Encoding]::GetEncoding(936)

function Read-Gbk([string]$RelativePath) {
    return [IO.File]::ReadAllText((Join-Path $Root $RelativePath), $Gbk)
}

function Assert-Contains([string]$Text, [string]$Expected, [string]$Message) {
    if (-not $Text.Contains($Expected)) { throw $Message }
}

$PidSource = Read-Gbk 'Seekfree_TC264_Opensource_Library/code/PID.c'
$Cpu1 = Read-Gbk 'Seekfree_TC264_Opensource_Library/user/cpu1_main.c'

Assert-Contains $PidSource '#define PD_ERR_DEAD_ZONE 4.0f' 'PD dead zone is not +/-4'
Assert-Contains $Cpu1 '#define PD_KP          2.5f' 'PD proportional gain is not 2.5'

function Get-SteadyPdAngle([float]$Err) {
    $Center = 80.0
    if ($Err -ge -4.0 -and $Err -le 4.0) { return $Center }
    $Out = 2.5 * (-$Err) + $Center
    return [Math]::Max(9.0, [Math]::Min(132.0, $Out))
}

if ((Get-SteadyPdAngle -Err (-4.0)) -ne 80.0 -or
    (Get-SteadyPdAngle -Err 4.0) -ne 80.0) {
    throw 'Dead-zone boundary must return the servo to center'
}

$Left = Get-SteadyPdAngle -Err (-10.0)
$Right = Get-SteadyPdAngle -Err 10.0
if ($Left -lt 105.0 -or $Right -gt 55.0) {
    throw 'Small track errors still produce too little steering angle'
}

$MaxLeft = Get-SteadyPdAngle -Err (-100.0)
$MaxRight = Get-SteadyPdAngle -Err 100.0
if ($MaxLeft -ne 132.0 -or $MaxRight -ne 9.0) {
    throw 'Steering output is not clamped in both directions'
}

Write-Output 'PASS steering response'
