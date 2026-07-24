$ErrorActionPreference = 'Stop'
$Root = Split-Path -Parent $PSScriptRoot
$Gbk = [Text.Encoding]::GetEncoding(936)

function Read-Gbk([string]$RelativePath) {
    return [IO.File]::ReadAllText((Join-Path $Root $RelativePath), $Gbk)
}

function Assert-Contains([string]$Text, [string]$Expected, [string]$Message) {
    if (-not $Text.Contains($Expected)) { throw $Message }
}

$Camera = Read-Gbk 'Seekfree_TC264_Opensource_Library/code/Camera.c'
$Header = Read-Gbk 'Seekfree_TC264_Opensource_Library/code/Camera.h'
$Cpu0 = Read-Gbk 'Seekfree_TC264_Opensource_Library/user/cpu0_main.c'

@{
    RING_STATE_IDLE = 0; RING_STATE_CONFIRM = 1; RING_STATE_APPROACH = 2
    RING_STATE_ENTRY = 3; RING_STATE_INSIDE = 4; RING_STATE_EXIT = 5
    RING_STATE_RECOVERY = 6
}.GetEnumerator() | ForEach-Object {
    Assert-Contains $Header ("#define {0}" -f $_.Key) ("Missing ring state {0}" -f $_.Key)
}

Assert-Contains $Header '#define RING_CONFIRM_FRAMES' 'Missing ring confirmation frame limit'
Assert-Contains $Header '#define RING_EXIT_CONFIRM_FRAMES' 'Missing exit confirmation frame limit'
Assert-Contains $Header '#define RING_EXIT_STABLE_FRAMES' 'Missing exit stable frame limit'
Assert-Contains $Header '#define RING_RECOVERY_FRAMES' 'Missing recovery lockout frame limit'
# ponytail: RING_ENTRY_CENTER_OFFSET removed, uses FILL_ENTRY_OFFSET instead
# ponytail: RING_INSIDE_CENTER_OFFSET removed, uses FILL_INSIDE_OFFSET instead

Assert-Contains $Camera 'static void Ring_Set_State(uint8 state)' 'Ring state transition helper is missing'
Assert-Contains $Camera 'static int Ring_Find_Valley_Point' 'Valley point detector is missing'
Assert-Contains $Camera 'static uint8 Ring_Has_Exit_Feature' 'Exit feature detector is missing'
Assert-Contains $Camera 'static void Ring_Rebuild_Fill' 'Ring fill rebuilding is missing'
Assert-Contains $Camera 'RING_STATE_CONFIRM' 'Detection does not enter confirmation state'
Assert-Contains $Camera 's_ring_exit_loss_seen' 'Exit-side loss history is not persistent'
Assert-Contains $Camera 'RING_INSIDE_MAX_FRAMES' 'Inside timeout guard is missing'
Assert-Contains $Camera 'RING_EXIT_MAX_FRAMES' 'Exit timeout guard is missing'

$RingStateStart = $Camera.IndexOf('static void Ring_State_Update(void)')
$RingStateEnd = $Camera.IndexOf('void Element_Judgment_Left_Rings(void)', $RingStateStart)
if ($RingStateStart -lt 0 -or $RingStateEnd -le $RingStateStart) {
    throw 'Ring state update function block is missing'
}
$RingStateCode = $Camera.Substring($RingStateStart, $RingStateEnd - $RingStateStart)
Assert-Contains $RingStateCode 'case RING_STATE_CONFIRM:' 'Confirm transition is missing'
Assert-Contains $RingStateCode 'case RING_STATE_APPROACH:' 'Approach transition is missing'
Assert-Contains $RingStateCode 'case RING_STATE_ENTRY:' 'Entry transition is missing'
Assert-Contains $RingStateCode 'case RING_STATE_INSIDE:' 'Inside transition is missing'
Assert-Contains $RingStateCode 'case RING_STATE_EXIT:' 'Exit transition is missing'
Assert-Contains $RingStateCode 'case RING_STATE_RECOVERY:' 'Recovery transition is missing'

$ApproachStart = $RingStateCode.IndexOf('case RING_STATE_APPROACH:')
$EntryStart = $RingStateCode.IndexOf('case RING_STATE_ENTRY:', $ApproachStart)
$ApproachCode = $RingStateCode.Substring($ApproachStart, $EntryStart - $ApproachStart)
Assert-Contains $ApproachCode 's_ring_state_frames >= 3U' 'APPROACH can advance before three frames'

$Exit2Start = $RingStateCode.IndexOf('case RING_STATE_EXIT2:')
$ExitStart = $RingStateCode.IndexOf('case RING_STATE_EXIT:', $Exit2Start)
if ($Exit2Start -lt 0 -or $ExitStart -le $Exit2Start) {
    throw 'EXIT2 state block is missing'
}
$Exit2Code = $RingStateCode.Substring($Exit2Start, $ExitStart - $Exit2Start)
Assert-Contains $Exit2Code 's_ring_exit2_miss_frames == 0U' 'EXIT2 row jump is not restricted to adjacent valid frames'
Assert-Contains $Exit2Code 'c2r - s_ring_exit1_corner2_row > 20' 'EXIT2 does not require a row increase greater than 20'
Assert-Contains $Exit2Code 'if (exit2_row_jump)' 'EXIT2 does not advance directly from the row jump'
Assert-Contains $Camera 'uint8 Ring_Should_Hold_Err(void)' 'Missing EXIT2 jump-frame Err hold helper'
Assert-Contains $Camera 'ImageFlag.image_element_rings_flag == RING_STATE_EXIT' 'Err hold is not restricted to EXIT'
Assert-Contains $Camera 's_ring_state_frames == 0U' 'Err hold is not restricted to the transition frame'
Assert-Contains $Header 'uint8 Ring_Should_Hold_Err(void);' 'Err hold helper is not exposed to CPU0'
Assert-Contains $Cpu0 'if (!Ring_Should_Hold_Err())' 'CPU0 still overwrites Err on the EXIT2 jump frame'
Assert-Contains $Exit2Code 'g_corner_black_max = (c2r >= 0) ? c2r : 99;' 'EXIT2 row is not exposed through CB'
Assert-Contains $Exit2Code 'g_bottom_black_width = (int)s_ring_exit2_miss_frames;' 'EXIT2 miss count is not exposed through BW'
Assert-Contains $Exit2Code 'g_ring_miss_cnt = (int)s_ring_state_frames;' 'EXIT2 state frame count is not exposed through MS'
if ($Exit2Code.Contains('RING_EXIT2_PASS_ROW') -or
    $Exit2Code.Contains('RING_EXIT2_MAX_FRAMES') -or
    $Exit2Code.Contains('s_ring_feature_count')) {
    throw 'EXIT2 still contains an old completion condition'
}
if ($Exit2Code.Contains('Ring_Is_Stable_Road()')) {
    throw 'EXIT2 still advances from generic stable-road detection'
}

function Test-Exit2RowJump([int]$PreviousRow, [int]$CurrentRow, [int]$MissFrames) {
    return ($PreviousRow -ge 0 -and $CurrentRow -ge 0 -and
            $MissFrames -eq 0 -and $CurrentRow - $PreviousRow -gt 20)
}
if (Test-Exit2RowJump 30 50 0) { throw 'EXIT2 accepts a row increase of only 20' }
if (-not (Test-Exit2RowJump 30 51 0)) { throw 'EXIT2 rejects a row increase of 21' }
if (Test-Exit2RowJump 30 55 1) { throw 'EXIT2 accepts a jump across a missing frame' }

$ExitDetectorStart = $Camera.IndexOf('static uint8 Ring_Find_Exit1_Corners')
$ExitDetectorEnd = $Camera.IndexOf('static void Ring_Update_Exit_Point', $ExitDetectorStart)
$ExitDetectorCode = $Camera.Substring($ExitDetectorStart, $ExitDetectorEnd - $ExitDetectorStart)
Assert-Contains $ExitDetectorCode 'Pixle[row + 1][col] == IMG_WHITE' 'EXIT2 detector does not require white below the black endpoint'
Assert-Contains $ExitDetectorCode 'Pixle[row + 2][col] == IMG_WHITE' 'EXIT2 detector does not reject a continuous side black area'

$FillStart = $Camera.IndexOf('static void Ring_Rebuild_Fill(uint8 direction)')
$FillEnd = $Camera.IndexOf('static void Ring_State_Update(void)', $FillStart)
$FillCode = $Camera.Substring($FillStart, $FillEnd - $FillStart)
if ($FillCode.Contains('if (ring_state == RING_STATE_EXIT && s_ring_state_frames == 0U) return;')) {
    throw 'EXIT2 jump frame incorrectly skips ring fill'
}
$EntryFillStart = $FillCode.IndexOf('case RING_STATE_ENTRY:')
$Exit1FillStart = $FillCode.IndexOf('case RING_STATE_EXIT1:', $EntryFillStart)
$EntryFillCode = $FillCode.Substring($EntryFillStart, $Exit1FillStart - $EntryFillStart)
Assert-Contains $EntryFillCode 'for (row = s_ring_entry_corner_row - 1; row > ImageStatus.OFFLine; row--)' 'ENTRY does not scan rows above the entry corner'
Assert-Contains $EntryFillCode 'black_segment_seen = 1;' 'ENTRY does not remember the white-to-black transition'
Assert-Contains $EntryFillCode 'black_segment_seen && Pixle[row][col] == IMG_WHITE' 'ENTRY does not wait for black-to-white after entering the black area'
Assert-Contains $EntryFillCode 'Pixle[row][col - 1] == IMG_BLACK' 'Right ring ENTRY does not finish on the second white area'
Assert-Contains $EntryFillCode 'ImageDeal[row].LeftBorder = col;' 'Right ring ENTRY does not use the final white point as the left border'
Assert-Contains $EntryFillCode 'Pixle[row][col + 1] == IMG_BLACK' 'Left ring ENTRY does not mirror the final black-to-white transition'
Assert-Contains $EntryFillCode 'ImageDeal[row].RightBorder = col;' 'Left ring ENTRY does not use the final white point as the right border'

$Exit2FillStart = $FillCode.IndexOf('case RING_STATE_EXIT2:')
$InsideFillStart = $FillCode.IndexOf('case RING_STATE_INSIDE:', $Exit2FillStart)
$Exit2FillCode = $FillCode.Substring($Exit2FillStart, $InsideFillStart - $Exit2FillStart)
Assert-Contains $Exit2FillCode 'ImageSensorMid + Half_Bend_Wide[SCAN_BASE_START_ROW] * 2 / 3' 'Left ring EXIT2 anchor does not produce a leftward center'
Assert-Contains $Exit2FillCode 'ImageSensorMid - Half_Bend_Wide[SCAN_BASE_START_ROW] * 2 / 3' 'Right ring EXIT2 anchor does not produce a rightward center'
if ($Exit2FillCode.Contains('SCAN_BASE_START_ROW, LCDW - 1') -or
    $Exit2FillCode.Contains('SCAN_BASE_START_ROW, 0')) {
    throw 'EXIT2 still anchors the fill line to an image edge'
}

$NormalExitStart = $FillCode.IndexOf('case RING_STATE_EXIT:')
$RecoveryStart = $FillCode.IndexOf('case RING_STATE_RECOVERY:', $NormalExitStart)
$NormalExitCode = $FillCode.Substring($NormalExitStart, $RecoveryStart - $NormalExitStart)
Assert-Contains $NormalExitCode 'ImageDeal[row].RightBorder' 'Left ring EXIT fill does not use the right border'

function New-RingModel {
    return [pscustomobject]@{
        State = 1; StateFrames = 0; Confirm = 0; Evidence = 0
        Stable = 0; LossSeen = $false
    }
}

function Step-RingModel($Model, [bool]$Candidate, [bool]$EntryCorner,
                        [bool]$InsideVisual, [bool]$ExitLoss,
                        [bool]$ExitFeature, [bool]$StableRoad) {
    $Model.StateFrames++
    switch ($Model.State) {
        1 {
            if ($Candidate) { $Model.Confirm++ } else { $Model.Confirm = 0 }
            if ($Model.Confirm -ge 3) { $Model.State = 2; $Model.StateFrames = 0 }
            elseif ($Model.StateFrames -ge 8) { $Model.State = 0 }
        }
        2 {
            if ($EntryCorner) { $Model.Evidence++ } else { $Model.Evidence = 0 }
            if (($Model.StateFrames -ge 3 -and $Model.Evidence -ge 2) -or
                $Model.StateFrames -ge 24) {
                $Model.State = 3; $Model.StateFrames = 0; $Model.Evidence = 0
            }
        }
        3 {
            if ($InsideVisual) { $Model.Evidence++ } else { $Model.Evidence = 0 }
            if ($Model.Evidence -ge 3 -or $Model.StateFrames -ge 30) {
                $Model.State = 4; $Model.StateFrames = 0; $Model.Evidence = 0
            }
        }
        4 {
            if ($ExitLoss) { $Model.LossSeen = $true }
            if ($Model.LossSeen -and $ExitFeature) { $Model.Evidence++ }
            else { $Model.Evidence = 0 }
            if ($Model.Evidence -ge 2 -or $Model.StateFrames -ge 90) {
                $Model.State = 5; $Model.StateFrames = 0; $Model.Stable = 0
            }
        }
        5 {
            if ($StableRoad) { $Model.Stable++ } else { $Model.Stable = 0 }
            if ($Model.Stable -ge 8 -or $Model.StateFrames -ge 60) {
                $Model.State = 6; $Model.StateFrames = 0; $Model.Stable = 0
            }
        }
        6 {
            if ($StableRoad) { $Model.Stable++ } else { $Model.Stable = 0 }
            if (($Model.StateFrames -ge 12 -and $Model.Stable -ge 4) -or
                $Model.StateFrames -ge 40) { $Model.State = 0 }
        }
    }
}

$M = New-RingModel
Step-RingModel $M $true $false $false $false $false $false
if ($M.State -ne 1) { throw 'Ring confirms from only one additional frame' }
Step-RingModel $M $true $false $false $false $false $false
if ($M.State -ne 1) { throw 'Ring confirms from only two frames' }
Step-RingModel $M $true $false $false $false $false $false
if ($M.State -ne 2) { throw 'Ring does not advance after three confirmed frames' }
1..2 | ForEach-Object { Step-RingModel $M $true $true $false $false $false $false }
if ($M.State -ne 2) { throw 'Approach advances before three frames' }
Step-RingModel $M $true $true $false $false $false $false
if ($M.State -ne 3) { throw 'Approach does not advance on the third frame' }
1..3 | ForEach-Object { Step-RingModel $M $false $false $true $false $false $true }
if ($M.State -ne 4) { throw 'Entry does not advance to inside state' }

Step-RingModel $M $false $false $false $false $true $true
if ($M.State -ne 4) { throw 'Transient line recovery skips the inside state' }
Step-RingModel $M $false $false $false $true $false $false
1..2 | ForEach-Object { Step-RingModel $M $false $false $false $false $true $false }
if ($M.State -ne 5) { throw 'Confirmed exit feature does not advance to exit state' }
1..8 | ForEach-Object { Step-RingModel $M $false $false $false $false $false $true }
if ($M.State -ne 6) { throw 'Stable exit does not advance to recovery' }
1..12 | ForEach-Object { Step-RingModel $M $false $false $false $false $false $true }
if ($M.State -ne 0) { throw 'Recovery lockout does not release the ring' }

$M = New-RingModel
$M.State = 4; $M.StateFrames = 89
Step-RingModel $M $false $false $false $false $false $false
if ($M.State -ne 5) { throw 'Inside timeout can leave the machine stuck in the ring' }
$M.StateFrames = 59
Step-RingModel $M $false $false $false $false $false $false
if ($M.State -ne 6) { throw 'Exit timeout can leave the machine stuck at the exit' }

Write-Output 'PASS camera seven-state ring machine'
