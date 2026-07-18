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

$Shared = Read-Gbk 'Seekfree_TC264_Opensource_Library/code/Shared.h'
$Cpu0 = Read-Gbk 'Seekfree_TC264_Opensource_Library/user/cpu0_main.c'
$Cpu1 = Read-Gbk 'Seekfree_TC264_Opensource_Library/user/cpu1_main.c'

Assert-Contains $Shared 'extern volatile uint8_t StopRequest;' 'Shared stop request is missing'
Assert-Contains $Cpu0 'volatile uint8_t StopRequest = 0U;' 'CPU0 stop request storage is missing'
Assert-Contains $Cpu0 'if (ImageFlag.Zebra_Flag != 0)' 'CPU0 does not latch a zebra detection'
Assert-Contains $Cpu0 'StopRequest = 1U;' 'CPU0 does not request a stop after zebra detection'
Assert-NotContains $Cpu1 'StopRequest = 0U;' 'CPU1 must not clear the latched stop request'

$StopBranch = $Cpu1.IndexOf('if (StopRequest != 0U)')
$PiUpdate = $Cpu1.IndexOf('pwm_left  = PI_Update')
if ($StopBranch -lt 0 -or $PiUpdate -lt 0 -or $StopBranch -gt $PiUpdate) {
    throw 'CPU1 stop branch must run before the speed PI update'
}

$StopEnd = $Cpu1.IndexOf('continue;', $StopBranch)
if ($StopEnd -lt 0 -or $StopEnd -gt $PiUpdate) {
    throw 'CPU1 stop branch does not bypass the speed PI update'
}
$StopCode = $Cpu1.Substring($StopBranch, $StopEnd - $StopBranch)
Assert-Contains $StopCode 'Motor_SetLeftPWM(0);' 'Left motor is not forced to zero'
Assert-Contains $StopCode 'Motor_SetRightPWM(0);' 'Right motor is not forced to zero'
Assert-Contains $StopCode 'Servo_SetAngleDeg(SERVO_CENTER_ANGLE);' 'Servo is not centered while stopped'

function Update-StopRequest([int]$Current, [int]$ZebraFlag) {
    if ($ZebraFlag -ne 0) { return 1 }
    return $Current
}

if ((Update-StopRequest 0 0) -ne 0) { throw 'Stop request triggers without a zebra' }
if ((Update-StopRequest 0 1) -ne 1) { throw 'Zebra detection does not trigger a stop' }
if ((Update-StopRequest 1 0) -ne 1) { throw 'Stop request is not latched until reset' }

Write-Output 'PASS zebra latched stop'
