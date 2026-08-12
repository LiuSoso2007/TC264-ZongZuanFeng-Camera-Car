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

function Test-LockProtocol([string]$Text) {
    $Options = [Text.RegularExpressions.RegexOptions]::Singleline
    $PublishPattern = 'static\s+inline\s+void\s+Shared_PublishErr\s*\(float err,\s*uint8_t ring_entry_slowdown\).*?' +
        'while\s*\(IfxCpu_acquireMutex\(&ErrMailboxLock\)\s*==\s*FALSE\).*?' +
        'Err\s*=\s*err;\s*RingEntrySlowdown\s*=\s*ring_entry_slowdown;\s*ErrReady\s*=\s*1U;\s*IfxCpu_releaseMutex\(&ErrMailboxLock\);'
    $TakePattern = 'static\s+inline\s+uint8_t\s+Shared_TakeErr\s*\(float \*err,\s*uint8_t \*ring_entry_slowdown\).*?' +
        'if\s*\(IfxCpu_acquireMutex\(&ErrMailboxLock\)\s*!=\s*FALSE\).*?' +
        'if\s*\(ErrReady\s*!=\s*0U\).*?\*err\s*=\s*Err;\s*\*ring_entry_slowdown\s*=\s*RingEntrySlowdown;\s*ErrReady\s*=\s*0U;\s*' +
        'has_new_err\s*=\s*1U;.*?IfxCpu_releaseMutex\(&ErrMailboxLock\);.*?return\s+has_new_err;'
    return [regex]::IsMatch($Text, $PublishPattern, $Options) -and
           [regex]::IsMatch($Text, $TakePattern, $Options)
}

if (-not (Test-LockProtocol -Text $Shared)) {
    throw 'Shared.h mailbox lock/write/clear/release order is invalid'
}

# Mutation checks prove the source validator catches lost-ready and lost-release defects.
if (Test-LockProtocol -Text $Shared.Replace('ErrReady = 1U;', 'ErrReady = 0U;')) {
    throw 'Source validator accepted a broken publish-ready assignment'
}
if (Test-LockProtocol -Text $Shared.Replace('IfxCpu_releaseMutex(&ErrMailboxLock);', '')) {
    throw 'Source validator accepted a mailbox that never releases its lock'
}

Assert-Contains $PidHeader 'void PD_Update(float Kp, float Kd, float err);' 'PD API does not accept a local Err snapshot'
Assert-Contains $PidSource 'void PD_Update(float Kp, float Kd, float err)' 'PD implementation does not accept a local Err snapshot'
Assert-Contains $PidSource 's_pd_err0 = err;' 'PD does not use the stable local Err snapshot'
Assert-Contains $Cpu0 'Shared_PublishErr(frame_err, ring_entry_slowdown);' 'CPU0 does not publish Err and ring slowdown together after a completed frame'
Assert-Contains $Cpu1 'if (Shared_TakeErr(&new_position_err, &new_ring_entry_slowdown))' 'CPU1 does not consume Err and ring slowdown with the same new-data guard'
Assert-Contains $Cpu1 'ring_entry_slowdown = new_ring_entry_slowdown;' 'CPU1 does not update the local slowdown snapshot from the mailbox'
Assert-Contains $Cpu1 'PD_Update(PD_KP, PD_KD, position_err);' 'CPU1 does not run PD from one stable Err snapshot'

$Cpu1PdPattern = 'if\s*\(has_new_err\s*!=\s*0U\s*&&\s*StopRequest\s*==\s*0U\)\s*\{\s*PD_Update\(PD_KP, PD_KD, position_err\);\s*\}'
if (-not [regex]::IsMatch($Cpu1, $Cpu1PdPattern)) {
    throw 'CPU1 PD call is not guarded by new Err and the stop request'
}

function New-MailboxState {
    return [pscustomobject]@{ Owner = ''; Ready = $false; FrameId = -1; Slowdown = 0 }
}

function Try-PublishMailbox([object]$State, [int]$FrameId, [int]$Slowdown) {
    if ($State.Owner -ne '') { return $false }
    $State.Owner = 'CPU0'
    $State.FrameId = $FrameId
    $State.Slowdown = $Slowdown
    $State.Ready = $true
    $State.Owner = ''
    return $true
}

function Try-TakeMailbox([object]$State) {
    if ($State.Owner -ne '') { return $null }
    $State.Owner = 'CPU1'
    $Result = $null
    if ($State.Ready) {
        $Result = [pscustomobject]@{ FrameId = $State.FrameId; Slowdown = $State.Slowdown }
        $State.Ready = $false
    }
    $State.Owner = ''
    return $Result
}

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
    $Mailbox = New-MailboxState
    $LatestFrameId = -1
    $LastConsumedId = -1
    $LastSlowdown = 0
    $SlowdownStuckAfterExit = $false
    $OldLastFrameId = -1
    $Published = 0
    $Consumed = 0
    $Duplicate = 0
    $MaxLatencyUs = 0
    $OldPdCalls = 0
    $OldDuplicate = 0
    $OldMaxFirstLatencyUs = 0
    $PublishTimes = @{}

    foreach ($Event in $Events) {
        if ($Event.Kind -eq 'Publish') {
            $LatestFrameId = $Event.FrameId
            $Slowdown = if (($LatestFrameId % 6) -lt 3) { 1 } else { 0 }
            if (-not (Try-PublishMailbox -State $Mailbox -FrameId $LatestFrameId -Slowdown $Slowdown)) {
                throw 'Unexpected producer lock collision in the regular event stream'
            }
            $PublishTimes[$LatestFrameId] = $Event.TimeUs
            $Published++
            continue
        }

        # Old code ran PD on every 10 ms tick, including repeated reads of the same frame.
        if ($LatestFrameId -ge 0) {
            $OldPdCalls++
            if ($LatestFrameId -eq $OldLastFrameId) {
                $OldDuplicate++
            }
            else {
                $OldLatencyUs = $Event.TimeUs - $PublishTimes[$LatestFrameId]
                if ($OldLatencyUs -gt $OldMaxFirstLatencyUs) { $OldMaxFirstLatencyUs = $OldLatencyUs }
                $OldLastFrameId = $LatestFrameId
            }
        }

        $Taken = Try-TakeMailbox -State $Mailbox
        if ($null -ne $Taken) {
            if ($Taken.FrameId -le $LastConsumedId) { $Duplicate++ }
            $LatencyUs = $Event.TimeUs - $PublishTimes[$Taken.FrameId]
            if ($LatencyUs -gt $MaxLatencyUs) { $MaxLatencyUs = $LatencyUs }
            $LastConsumedId = $Taken.FrameId
            $LastSlowdown = $Taken.Slowdown
            if (($Taken.FrameId % 6) -ge 3 -and $LastSlowdown -ne 0) { $SlowdownStuckAfterExit = $true }
            $Consumed++
        }
    }

    return [pscustomobject]@{
        Published = $Published
        Consumed = $Consumed
        Duplicate = $Duplicate
        MaxLatencyUs = $MaxLatencyUs
        LastConsumedId = $LastConsumedId
        OldPdCalls = $OldPdCalls
        OldDuplicate = $OldDuplicate
        OldMaxFirstLatencyUs = $OldMaxFirstLatencyUs
        SlowdownStuckAfterExit = $SlowdownStuckAfterExit
    }
}

$Events = Get-Events -DurationUs 2000000 -FrameIntervalsUs @(16000, 30000, 18000, 27000, 22000, 25000, 17000, 29000) -ControlPeriodUs 10000
$Result = Invoke-MailboxSimulation -Events $Events

if ($Result.Duplicate -ne 0) { throw "Mailbox duplicated $($Result.Duplicate) samples" }
if ($Result.Consumed -lt ($Result.Published - 1)) { throw 'The 10 ms consumer dropped more than the final pending frame' }
if ($Result.MaxLatencyUs -gt 10000) { throw "Publish-to-consume latency exceeded 10 ms: $($Result.MaxLatencyUs) us" }
if ($Result.OldDuplicate -le 0) { throw 'Old 10 ms loop model did not reproduce duplicate PD updates' }
if ($Result.OldMaxFirstLatencyUs -ne $Result.MaxLatencyUs) {
    throw 'Mailbox must keep the old 10 ms first-sample latency rather than claim a false speedup'
}
if ($Result.SlowdownStuckAfterExit) { throw 'Ring slowdown stayed enabled after a non-entry frame was consumed' }

# The first new Err has a derivative term; reusing the same Err on the next tick makes that term zero.
$Kd = 1.15
$PreviousErr = 0.0
$StepErr = 10.0
$FirstDerivative = $Kd * ($StepErr - $PreviousErr)
$RepeatedDerivative = $Kd * ($StepErr - $StepErr)
if ($FirstDerivative -eq 0.0 -or $RepeatedDerivative -ne 0.0) {
    throw 'PD duplicate model did not reproduce derivative cancellation'
}

# Producer overload must overwrite stale frames instead of making CPU1 follow a delayed FIFO backlog.
$OverloadEvents = Get-Events -DurationUs 500000 -FrameIntervalsUs @(4000, 5000, 6000) -ControlPeriodUs 10000
$Overload = Invoke-MailboxSimulation -Events $OverloadEvents
if ($Overload.Consumed -ge $Overload.Published) { throw 'Overload model did not exercise latest-value overwrite' }
if ($Overload.Duplicate -ne 0) { throw 'Overload model consumed one frame more than once' }
if ($Overload.MaxLatencyUs -gt 10000) { throw 'Overload model delivered a stale frame after one control interval' }

# Lock collision, producer retry, latest overwrite and no-frame behavior all use the same lock state model.
$Collision = New-MailboxState
$Collision.Owner = 'CPU0'
$Collision.Ready = $true
$Collision.FrameId = 7
$Collision.Slowdown = 1
if ($null -ne (Try-TakeMailbox -State $Collision)) { throw 'CPU1 must not block or read while CPU0 owns the lock' }
$Collision.Owner = ''
$CollisionTaken = Try-TakeMailbox -State $Collision
if ($CollisionTaken.FrameId -ne 7 -or $CollisionTaken.Slowdown -ne 1) { throw 'CPU1 did not retry the pending frame after lock release' }
if ($null -ne (Try-TakeMailbox -State $Collision)) { throw 'CPU1 consumed the retried frame twice' }

$ProducerRetry = New-MailboxState
$ProducerRetry.Owner = 'CPU1'
if (Try-PublishMailbox -State $ProducerRetry -FrameId 9 -Slowdown 1) { throw 'CPU0 publish entered while CPU1 owned the lock' }
$ProducerRetry.Owner = ''
if (-not (Try-PublishMailbox -State $ProducerRetry -FrameId 9 -Slowdown 0)) { throw 'CPU0 publish retry failed after lock release' }
$ProducerTaken = Try-TakeMailbox -State $ProducerRetry
if ($ProducerTaken.FrameId -ne 9 -or $ProducerTaken.Slowdown -ne 0) { throw 'Retried CPU0 publish was not delivered' }

$LatestOnly = New-MailboxState
$null = Try-PublishMailbox -State $LatestOnly -FrameId 1 -Slowdown 1
$null = Try-PublishMailbox -State $LatestOnly -FrameId 2 -Slowdown 1
$null = Try-PublishMailbox -State $LatestOnly -FrameId 3 -Slowdown 0
$LatestTaken = Try-TakeMailbox -State $LatestOnly
if ($LatestTaken.FrameId -ne 3) { throw 'Mailbox did not overwrite stale frames with the latest value' }
if ($LatestTaken.Slowdown -ne 0) { throw 'Mailbox did not overwrite stale ring slowdown state with the latest frame' }
if ($null -ne (Try-TakeMailbox -State $LatestOnly)) { throw 'Latest-only mailbox created a duplicate after overwrite' }

$NoFrame = New-MailboxState
if ($null -ne (Try-TakeMailbox -State $NoFrame)) { throw 'No-frame model generated a false Err update' }

Write-Output ("PASS TC264 Err mailbox: frames={0}, old_pd={1}, old_duplicate={2}, new_pd={3}, new_duplicate={4}, same_first_latency_us={5}, overload={6}/{7}" -f `
    $Result.Published, $Result.OldPdCalls, $Result.OldDuplicate, $Result.Consumed, `
    $Result.Duplicate, $Result.MaxLatencyUs, $Overload.Consumed, $Overload.Published)
