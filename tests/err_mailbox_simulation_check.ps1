$ErrorActionPreference = 'Stop'
$Root = Split-Path -Parent $PSScriptRoot
$Gbk = [Text.Encoding]::GetEncoding(936)

function Read-Gbk([string]$RelativePath) {
    return [IO.File]::ReadAllText((Join-Path $Root $RelativePath), $Gbk)
}

function Assert-Contains([string]$Text, [string]$Expected, [string]$Message) {
    if (-not $Text.Contains($Expected)) { throw $Message }
}

$Shared = Read-Gbk 'Seekfree_TC264_Opensource_Library/code/Shared.h'
$PidHeader = Read-Gbk 'Seekfree_TC264_Opensource_Library/code/PID.h'
$PidSource = Read-Gbk 'Seekfree_TC264_Opensource_Library/code/PID.c'
$Cpu0 = Read-Gbk 'Seekfree_TC264_Opensource_Library/user/cpu0_main.c'
$Cpu1 = Read-Gbk 'Seekfree_TC264_Opensource_Library/user/cpu1_main.c'

Assert-Contains $Shared 'Shared_PublishErr(float err)' 'Shared.h lacks the atomic CPU0 publish API'
Assert-Contains $Shared 'Shared_TakeErr(float *err)' 'Shared.h lacks the atomic CPU1 take API'
Assert-Contains $Shared 'IfxCpu_acquireMutex' 'Err mailbox does not use the TC264 iLLD mutex'
Assert-Contains $PidHeader 'void PD_Update(float Kp, float Kd, float err);' 'PD API does not accept a local Err snapshot'
Assert-Contains $PidSource 'void PD_Update(float Kp, float Kd, float err)' 'PD implementation does not accept a local Err snapshot'
Assert-Contains $Cpu0 'Shared_PublishErr(frame_err);' 'CPU0 does not publish Err after a completed frame'
Assert-Contains $Cpu1 'if (Shared_TakeErr(&new_position_err))' 'CPU1 does not consume Err with the new-data guard'
Assert-Contains $Cpu1 'PD_Update(PD_KP, PD_KD, position_err);' 'CPU1 does not run PD from one stable Err snapshot'

function Get-Events([int]$DurationUs, [int[]]$FrameIntervalsUs, [int]$ControlPeriodUs) {
    $Events = [Collections.Generic.List[object]]::new()
    $TimeUs = 3000
    $FrameId = 0
    while ($TimeUs -le $DurationUs) {
        $Events.Add([pscustomobject]@{ TimeUs = $TimeUs; Kind = 'Publish'; FrameId = $FrameId })
        $TimeUs += $FrameIntervalsUs[$FrameId % $FrameIntervalsUs.Count]
        $FrameId++
    }
    for ($TickUs = 0; $TickUs -le $DurationUs; $TickUs += $ControlPeriodUs) {
        $Events.Add([pscustomobject]@{ TimeUs = $TickUs; Kind = 'Control'; FrameId = -1 })
    }
    return $Events | Sort-Object TimeUs, @{ Expression = { if ($_.Kind -eq 'Publish') { 0 } else { 1 } } }
}

function Invoke-MailboxSimulation([object[]]$Events) {
    $Pending = $false
    $LatestFrameId = -1
    $LastConsumedId = -1
    $Published = 0
    $Consumed = 0
    $Duplicate = 0
    $MaxLatencyUs = 0
    $PublishTimes = @{}

    foreach ($Event in $Events) {
        if ($Event.Kind -eq 'Publish') {
            $LatestFrameId = $Event.FrameId
            $Pending = $true
            $PublishTimes[$LatestFrameId] = $Event.TimeUs
            $Published++
            continue
        }

        if ($Pending) {
            if ($LatestFrameId -le $LastConsumedId) { $Duplicate++ }
            $LatencyUs = $Event.TimeUs - $PublishTimes[$LatestFrameId]
            if ($LatencyUs -gt $MaxLatencyUs) { $MaxLatencyUs = $LatencyUs }
            $LastConsumedId = $LatestFrameId
            $Pending = $false
            $Consumed++
        }
    }

    return [pscustomobject]@{
        Published = $Published
        Consumed = $Consumed
        Duplicate = $Duplicate
        MaxLatencyUs = $MaxLatencyUs
        LastConsumedId = $LastConsumedId
    }
}

$Events = Get-Events -DurationUs 2000000 -FrameIntervalsUs @(16000, 30000, 18000, 27000, 22000, 25000, 17000, 29000) -ControlPeriodUs 10000
$Result = Invoke-MailboxSimulation -Events $Events

if ($Result.Duplicate -ne 0) { throw "Mailbox duplicated $($Result.Duplicate) samples" }
if ($Result.Consumed -lt ($Result.Published - 1)) { throw 'The 10 ms consumer dropped more than the final pending frame' }
if ($Result.MaxLatencyUs -gt 10000) { throw "Publish-to-consume latency exceeded 10 ms: $($Result.MaxLatencyUs) us" }

# Fixed 20 ms polling can wait almost 20 ms; mailbox polling is bounded to 10 ms without contention.
$Fixed20WorstUs = 20000
if ($Result.MaxLatencyUs -ge $Fixed20WorstUs) { throw 'Mailbox polling did not beat fixed 20 ms polling' }

# A frame published at 3 ms is handled at 10 ms by the mailbox, but at 20 ms by a fixed 20 ms loop.
$MailboxStepLatencyUs = 10000 - 3000
$Fixed20StepLatencyUs = 20000 - 3000
if (($Fixed20StepLatencyUs - $MailboxStepLatencyUs) -ne 10000) {
    throw 'The mailbox did not save one 10 ms control interval versus fixed 20 ms polling'
}

# Reprocessing the same Err at the next 10 ms tick erases the derivative boost before a 20 ms PWM latch.
$Center = 150.0
$Kp = 0.85
$Kd = 1.15
$StepErr = 10.0
$FirstPd = $Center + $Kp * $StepErr + $Kd * $StepErr
$RepeatedPd = $Center + $Kp * $StepErr
if ([Math]::Abs($FirstPd - $Center) -le [Math]::Abs($RepeatedPd - $Center)) {
    throw 'New-data-only PD did not preserve the first-frame derivative response'
}

# If CPU0 owns the lock exactly at a control tick, CPU1 skips instead of blocking and retries after 10 ms.
$ContendedPublishUs = 10000
$ContendedConsumeUs = 20000
if (($ContendedConsumeUs - $ContendedPublishUs) -gt 10000) {
    throw 'One lock collision delayed consumption by more than one control interval'
}

# Producer overload must overwrite stale frames instead of making CPU1 follow a delayed FIFO backlog.
$OverloadEvents = Get-Events -DurationUs 500000 -FrameIntervalsUs @(4000, 5000, 6000) -ControlPeriodUs 10000
$Overload = Invoke-MailboxSimulation -Events $OverloadEvents
if ($Overload.Consumed -ge $Overload.Published) { throw 'Overload model did not exercise latest-value overwrite' }
if ($Overload.Duplicate -ne 0) { throw 'Overload model consumed one frame more than once' }
if ($Overload.MaxLatencyUs -gt 10000) { throw 'Overload model delivered a stale frame after one control interval' }

# With no producer frames, control ticks must not invent a PD update.
$NoFrameEvents = 0..20 | ForEach-Object {
    [pscustomobject]@{ TimeUs = $_ * 10000; Kind = 'Control'; FrameId = -1 }
}
$NoFrame = Invoke-MailboxSimulation -Events $NoFrameEvents
if ($NoFrame.Consumed -ne 0 -or $NoFrame.Duplicate -ne 0) {
    throw 'No-frame model generated a false Err update'
}

function Get-NextPwmLatchUs([int]$ReadyUs, [int]$PhaseUs) {
    if ($ReadyUs -le $PhaseUs) { return $PhaseUs }
    return $PhaseUs + [int]([Math]::Ceiling(($ReadyUs - $PhaseUs) / 20000.0)) * 20000
}

# Sweep every 1 ms PWM phase. Mailbox data is ready at 10 ms; fixed polling is ready at 20 ms.
$MailboxLatencySumUs = 0
$FixedLatencySumUs = 0
$OldDerivativeLatchCount = 0
$NewDerivativeLatchCount = 0
for ($PhaseUs = 0; $PhaseUs -lt 20000; $PhaseUs += 1000) {
    $MailboxLatchUs = Get-NextPwmLatchUs -ReadyUs 10000 -PhaseUs $PhaseUs
    $FixedLatchUs = Get-NextPwmLatchUs -ReadyUs 20000 -PhaseUs $PhaseUs
    $MailboxLatencySumUs += $MailboxLatchUs - 3000
    $FixedLatencySumUs += $FixedLatchUs - 3000

    # Old 10 ms PD loses its derivative at the 20 ms duplicate update; mailbox keeps it until every latch.
    if ($MailboxLatchUs -lt 20000) { $OldDerivativeLatchCount++ }
    $NewDerivativeLatchCount++
}
$MailboxAverageLatchUs = $MailboxLatencySumUs / 20
$FixedAverageLatchUs = $FixedLatencySumUs / 20
if (($FixedAverageLatchUs - $MailboxAverageLatchUs) -ne 10000) {
    throw 'PWM phase sweep did not show the expected 10 ms average response gain'
}
if ($OldDerivativeLatchCount -ge $NewDerivativeLatchCount) {
    throw 'PWM phase sweep did not expose derivative overwrite in the old 10 ms loop'
}

Write-Output ("PASS TC264 Err mailbox simulation: published={0}, consumed={1}, duplicate={2}, max_latency_us={3}, overload={4}/{5}, avg_pwm_gain_us={6}, derivative_latches={7}->{8}" -f `
    $Result.Published, $Result.Consumed, $Result.Duplicate, $Result.MaxLatencyUs, `
    $Overload.Consumed, $Overload.Published, ($FixedAverageLatchUs - $MailboxAverageLatchUs), `
    $OldDerivativeLatchCount, $NewDerivativeLatchCount)
