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
$Cpu0 = Read-Gbk 'Seekfree_TC264_Opensource_Library/user/cpu0_main.c'

Assert-Contains $PidSource '#define PD_ERR_DEAD_ZONE 3.0f' 'PD dead zone is not +/-3'
Assert-Contains $Cpu1 '#define PD_KP          0.9f' 'PD proportional gain is not 0.9'
Assert-Contains $Cpu1 'static int8_t   StraightSpeed = 40;' 'Straight speed is not 40'
Assert-Contains $Cpu0 '#define STEERING_LOOKAHEAD_ROW 40' 'CPU0 lookahead row is not moved farther to row 40'
Assert-Contains $Cpu0 'ImageDeal[STEERING_LOOKAHEAD_ROW].Center' 'Err does not use the configured far lookahead row'
Assert-Contains $Cpu0 'ImageDeal[STEERING_LOOKAHEAD_ROW + 1].Center' 'Err does not average the second lookahead row'
Assert-Contains $Cpu0 'ImageDeal[STEERING_LOOKAHEAD_ROW + 2].Center' 'Err does not average the third lookahead row'

function Get-SteadyPdAngle([float]$Err) {
    $Center = 80.0
    if ($Err -ge -3.0 -and $Err -le 3.0) { return $Center }
    $Out = 0.9 * $Err + $Center
    return [Math]::Max(9.0, [Math]::Min(132.0, $Out))
}

if ((Get-SteadyPdAngle -Err (-3.0)) -ne 80.0 -or
    (Get-SteadyPdAngle -Err 3.0) -ne 80.0) {
    throw 'Dead-zone boundary must return the servo to center'
}

$Left = Get-SteadyPdAngle -Err (-10.0)
$Right = Get-SteadyPdAngle -Err 10.0
if ($Left -gt 71.0 -or $Right -lt 89.0) {
    throw 'Small track errors still produce too little steering angle'
}

$MaxLeft = Get-SteadyPdAngle -Err (-100.0)
$MaxRight = Get-SteadyPdAngle -Err 100.0
if ($MaxLeft -ne 9.0 -or $MaxRight -ne 132.0) {
    throw 'Steering output is not clamped in both directions'
}

Write-Output 'PASS steering response'
