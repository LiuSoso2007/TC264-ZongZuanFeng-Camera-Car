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

$Camera = Read-Gbk 'Seekfree_TC264_Opensource_Library/code/Camera.c'
$Cpu0 = Read-Gbk 'Seekfree_TC264_Opensource_Library/user/cpu0_main.c'
$ServoH = Read-Gbk 'Seekfree_TC264_Opensource_Library/code/Servo.h'
$ServoC = Read-Gbk 'Seekfree_TC264_Opensource_Library/code/Servo.c'
$PidSource = Read-Gbk 'Seekfree_TC264_Opensource_Library/code/PID.c'

Assert-Contains $Camera 'for (row = SCAN_BASE_START_ROW; (row - 2) > ImageStatus.OFFLine; row -= 2)' '蓝线仍可能连接OFFLine无效行'
Assert-NotContains $Cpu0 '/ (float)ImageSensorMid;' 'Err仍是归一化单位'
Assert-Contains $Cpu0 'Err = 0.0f;' '无有效赛道时未清零Err'
Assert-Contains $ServoH '#define SERVO_CENTER_ANGLE  80U' '舵机中位未统一为80度'
Assert-Contains $ServoH '#define SERVO_MIN_ANGLE      9U' '缺少舵机负方向限幅'
Assert-Contains $ServoH '#define SERVO_MAX_ANGLE    132U' '缺少舵机正方向限幅'
Assert-Contains $ServoC 'SERVO_CENTER_ANGLE' '舵机初始化未使用统一中位'
Assert-Contains $PidSource '+ (float)SERVO_CENTER_ANGLE;' 'PD未围绕统一中位输出'
Assert-Contains $PidSource 'Servo_SetAngleDeg(SERVO_CENTER_ANGLE);' 'PD死区未使用统一中位'

function Get-PdAngle([float]$Err) {
    $Center = 80.0
    if ($Err -ge -3.0 -and $Err -le 3.0) { return $Center }
    $Out = 1.5 * $Err + 0.4 * $Err + $Center
    return [Math]::Max(9.0, [Math]::Min(132.0, $Out))
}

$Negative = Get-PdAngle -Err (-10.0)
$Zero = Get-PdAngle -Err 0.0
$Positive = Get-PdAngle -Err 10.0
if (-not ($Negative -lt 80.0 -and $Zero -eq 80.0 -and $Positive -gt 80.0)) {
    throw 'PD未在中位两侧产生相反方向输出'
}
if ($Negative -lt 9.0 -or $Positive -gt 132.0) { throw 'PD输出越界' }

Write-Output 'PASS steering contract'
