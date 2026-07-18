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
$Motor = Read-Gbk 'Seekfree_TC264_Opensource_Library/code/Motor.c'
$Cpu0 = Read-Gbk 'Seekfree_TC264_Opensource_Library/user/cpu0_main.c'
$Cpu1 = Read-Gbk 'Seekfree_TC264_Opensource_Library/user/cpu1_main.c'

Assert-Contains $Shared 'extern volatile uint8_t StopRequest;' 'Shared stop request is missing'
Assert-Contains $Cpu0 'volatile uint8_t StopRequest = 0U;' 'CPU0 stop request storage is missing'
Assert-Contains $Cpu0 'if (ImageFlag.Zebra_Flag != 0)' 'CPU0 does not latch a zebra detection'
Assert-Contains $Cpu0 'StopRequest = 1U;' 'CPU0 does not request a stop after zebra detection'
Assert-NotContains $Cpu1 'StopRequest = 0U;' 'CPU1 must not clear the latched stop request'
Assert-Contains $Cpu0 '#define ZEBRA_STOP_CONFIRM_FRAMES 2U' 'Zebra detection is not debounced'
Assert-Contains $Cpu0 '#define ZEBRA_STOP_DELAY_FRAMES 8U' 'Finish-line run-on distance is missing'

$LeftMotor = $Motor.Substring($Motor.IndexOf('void Motor_SetLeftPWM'),
    $Motor.IndexOf('void Motor_SetRightPWM') - $Motor.IndexOf('void Motor_SetLeftPWM'))
$RightMotor = $Motor.Substring($Motor.IndexOf('void Motor_SetRightPWM'))
Assert-Contains $LeftMotor 'if (Speed == 0)' 'Left motor zero command does not use an explicit stop branch'
Assert-Contains $RightMotor 'if (Speed == 0)' 'Right motor zero command does not use an explicit stop branch'
Assert-Contains $LeftMotor 'pwm_set_duty(MOTOR_LEFT_IN1, 0);' 'Left motor IN1 is not cleared while stopped'
Assert-Contains $LeftMotor 'pwm_set_duty(MOTOR_LEFT_IN2, 0);' 'Left motor IN2 is not cleared while stopped'
Assert-Contains $RightMotor 'pwm_set_duty(MOTOR_RIGHT_IN1, 0);' 'Right motor IN1 is not cleared while stopped'
Assert-Contains $RightMotor 'pwm_set_duty(MOTOR_RIGHT_IN2, 0);' 'Right motor IN2 is not cleared while stopped'

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

function Update-ZebraStop([int]$Confirm, [int]$Delay, [int]$Pending, [int]$ZebraFlag) {
    if ($Pending -eq 0) {
        if ($ZebraFlag -ne 0) { $Confirm++ } else { $Confirm = 0 }
        if ($Confirm -ge 2) { $Pending = 1; $Delay = 0 }
    } elseif ($Delay -lt 8) {
        $Delay++
    } else {
        return @(1, $Confirm, $Delay, $Pending)
    }
    return @(0, $Confirm, $Delay, $Pending)
}

$State = @(0, 0, 0)
$Result = Update-ZebraStop $State[0] $State[1] $State[2] 1
if ($Result[0] -ne 0) { throw 'A single zebra frame stops the car too early' }
$Result = Update-ZebraStop $Result[1] $Result[2] $Result[3] 1
if ($Result[0] -ne 0) { throw 'Confirmed zebra must enter full-speed run-on first' }
for ($Frame = 0; $Frame -lt 8; $Frame++) {
    $Result = Update-ZebraStop $Result[1] $Result[2] $Result[3] 0
    if ($Result[0] -ne 0) { throw 'Car stops before the finish-line run-on completes' }
}
$Result = Update-ZebraStop $Result[1] $Result[2] $Result[3] 0
if ($Result[0] -ne 1) { throw 'Car does not stop after the finish-line run-on' }

Write-Output 'PASS zebra latched stop'
