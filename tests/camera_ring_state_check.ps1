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
$Header = Read-Gbk 'Seekfree_TC264_Opensource_Library/code/Camera.h'
$Cpu0 = Read-Gbk 'Seekfree_TC264_Opensource_Library/user/cpu0_main.c'

@{
    RING_STATE_IDLE = 0; RING_STATE_CONFIRM = 1; RING_STATE_APPROACH = 2
    RING_STATE_ENTRY = 3; RING_STATE_INSIDE = 4; RING_STATE_EXIT1 = 5
    RING_STATE_EXIT2 = 6; RING_STATE_RECOVERY = 7
}.GetEnumerator() | ForEach-Object {
    Assert-Contains $Header ("#define {0}" -f $_.Key) ("Missing ring state {0}" -f $_.Key)
}
Assert-NotContains $Header '#define RING_STATE_EXIT       ' 'Deleted EXIT state is still defined'

Assert-Contains $Header '#define RING_CONFIRM_FRAMES' 'Missing ring confirmation frame limit'
Assert-Contains $Header '#define RING_EXIT_CONFIRM_FRAMES' 'Missing exit confirmation frame limit'
Assert-NotContains $Header '#define RING_EXIT_STABLE_FRAMES' 'Deleted EXIT stable-frame limit is still defined'
Assert-NotContains $Header '#define RING_RECOVERY_FRAMES' 'RECOVERY still uses the old frame lockout'
# ponytail: RING_ENTRY_CENTER_OFFSET removed, uses FILL_ENTRY_OFFSET instead
# ponytail: RING_INSIDE_CENTER_OFFSET removed, uses FILL_INSIDE_OFFSET instead

Assert-Contains $Camera 'static void Ring_Set_State(uint8 state)' 'Ring state transition helper is missing'
Assert-Contains $Camera 'static int Ring_Find_Valley_Point' 'Valley point detector is missing'
Assert-Contains $Camera 'static uint8 Ring_Has_Exit_Feature' 'Exit feature detector is missing'
Assert-Contains $Camera 'static void Ring_Rebuild_Fill' 'Ring fill rebuilding is missing'
Assert-Contains $Camera 'RING_STATE_CONFIRM' 'Detection does not enter confirmation state'
Assert-Contains $Camera 's_ring_exit_loss_seen' 'Exit-side loss history is not persistent'
Assert-Contains $Camera 'RING_INSIDE_MAX_FRAMES' 'Inside timeout guard is missing'
Assert-NotContains $Camera 'RING_EXIT_MAX_FRAMES' 'Deleted EXIT timeout is still referenced'

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
Assert-Contains $RingStateCode 'case RING_STATE_RECOVERY:' 'Recovery transition is missing'
Assert-NotContains $RingStateCode 'case RING_STATE_EXIT:' 'Deleted EXIT transition is still present'

$ApproachStart = $RingStateCode.IndexOf('case RING_STATE_APPROACH:')
$EntryStart = $RingStateCode.IndexOf('case RING_STATE_ENTRY:', $ApproachStart)
$ApproachCode = $RingStateCode.Substring($ApproachStart, $EntryStart - $ApproachStart)
Assert-Contains $ApproachCode 's_ring_state_frames >= 3U' 'APPROACH can advance before three frames'
Assert-Contains $ApproachCode 'Ring_Set_State(RING_STATE_ENTRY);' 'APPROACH does not advance to ENTRY'

$Exit2Start = $RingStateCode.IndexOf('case RING_STATE_EXIT2:')
$RecoveryStart = $RingStateCode.IndexOf('case RING_STATE_RECOVERY:', $Exit2Start)
if ($Exit2Start -lt 0 -or $RecoveryStart -le $Exit2Start) {
    throw 'EXIT2 state block is missing'
}
$Exit2Code = $RingStateCode.Substring($Exit2Start, $RecoveryStart - $Exit2Start)
Assert-Contains $Exit2Code 's_ring_exit2_miss_frames == 0U' 'EXIT2 row jump is not restricted to adjacent valid frames'
Assert-Contains $Exit2Code 'c2r - s_ring_exit1_corner2_row > 20' 'EXIT2 does not require a row increase greater than 20'
Assert-Contains $Exit2Code 'if (exit2_row_jump)' 'EXIT2 does not advance directly from the row jump'
Assert-Contains $Exit2Code 'Ring_Set_State(RING_STATE_RECOVERY);' 'EXIT2 does not advance directly to RECOVERY'
Assert-NotContains $Exit2Code 'Ring_Set_State(RING_STATE_EXIT)' 'EXIT2 still advances through the deleted EXIT state'
Assert-Contains $Camera 'uint8 Ring_Should_Hold_Err(void)' 'Missing EXIT2 jump-frame Err hold helper'
Assert-Contains $Camera 'ImageFlag.image_element_rings_flag == RING_STATE_RECOVERY' 'Err hold is not restricted to the EXIT2-to-RECOVERY frame'
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

$RecoveryCode = $RingStateCode.Substring($RecoveryStart)
Assert-Contains $RecoveryCode 'Ring_Find_Entry_Corner(direction, &valley_row, &valley_col)' 'RECOVERY does not track exit corner 3'
Assert-Contains $RecoveryCode 'valley_row < 0 && s_ring_prev_exit3_row >= 0' 'RECOVERY does not end when exit corner 3 disappears'
Assert-Contains $RecoveryCode 's_ring_prev_exit3_row - valley_row > 20' 'RECOVERY does not end when exit corner 3 moves upward by more than 20 rows'
Assert-Contains $RecoveryCode 'Ring_Clear_State();' 'RECOVERY does not return directly to IDLE'
Assert-NotContains $RecoveryCode 'RING_RECOVERY_MAX_FRAMES' 'RECOVERY still uses the old timeout completion'
Assert-NotContains $RecoveryCode 'Ring_Is_Stable_Road()' 'RECOVERY still uses stable-road completion'

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

$Exit3FillStart = $Camera.IndexOf('static void Ring_Rebuild_Exit3_Border(uint8 direction)')
$Exit3FillEnd = $Camera.IndexOf('static void Ring_Rebuild_Fill(uint8 direction)', $Exit3FillStart)
if ($Exit3FillStart -lt 0 -or $Exit3FillEnd -le $Exit3FillStart) {
    throw 'Exit corner 3 fill helper is missing'
}
$Exit3FillCode = $Camera.Substring($Exit3FillStart, $Exit3FillEnd - $Exit3FillStart)
Assert-Contains $Exit3FillCode 'SCAN_BASE_START_ROW, LCDW - 1' 'Right ring does not connect the lower-right corner to exit corner 3'
Assert-Contains $Exit3FillCode "s_ring_exit3_corner_row, s_ring_exit3_corner_col, 'R'" 'Right ring does not rebuild the right border'
Assert-Contains $Exit3FillCode 'ImageDeal[row].RightBorder == LCDW - 1' 'Right ring does not rescan rows whose right border is at the image edge'
Assert-Contains $Exit3FillCode 'Pixle[row][col] == IMG_BLACK' 'Exit corner 3 fill does not scan the black segment'
Assert-Contains $Exit3FillCode 'Pixle[row][col] == IMG_WHITE' 'Exit corner 3 fill does not finish after black changes back to white'
Assert-Contains $Exit3FillCode 'ImageDeal[row].RightBorder = col;' 'Right ring does not replace the right border after white-black-white'
Assert-Contains $Exit3FillCode 'ImageDeal[row].LeftBorder == 0' 'Left ring does not mirror the edge rescan'
Assert-Contains $Exit3FillCode 'ImageDeal[row].LeftBorder = col;' 'Left ring does not replace the left border after white-black-white'

$RecoveryFillStart = $FillCode.IndexOf('case RING_STATE_RECOVERY:')
$RecoveryFillCode = $FillCode.Substring($RecoveryFillStart)
Assert-Contains $RecoveryFillCode 'Ring_Rebuild_Exit3_Border(direction);' 'RECOVERY does not use exit corner 3 fill'
Assert-NotContains $FillCode 'case RING_STATE_EXIT:' 'Deleted EXIT fill is still present'

Assert-Contains $Camera '"EX2T","RECV"' 'Camera RING display does not map state 7 to RECOVERY'
Assert-NotContains $Camera '"EX2T","EXIT","RECV"' 'Camera RING display still contains the deleted EXIT stage'
Assert-Contains $Camera 'rst < 8' 'Camera RING display still uses the old state count'
Assert-Contains $Cpu0 '"IDLE ","CNFM ","APRC ","ENTR ","INSD ","EXIT1","EXIT2","RECV "' 'CPU0 RING display is not fixed-width or does not map state 7 to RECOVERY'
Assert-NotContains $Cpu0 '"EXIT2","EXIT","RECV"' 'CPU0 RING display still contains the deleted EXIT stage'
Assert-Contains $Cpu0 'ring_st < 8U' 'CPU0 RING display still uses the old state count'
Assert-Contains $Cpu0 'ring_state < 8U' 'RF display still uses the old state count'
Assert-Contains $Cpu0 'rf = ring_state;' 'RF display does not report the actual ring state'
Assert-Contains $Cpu0 'ips200_show_string(50U, 190U, "---    ");' 'CPU0 RING idle display does not erase the full previous state'

function Test-RecoveryExit3End([int]$PreviousRow, [int]$CurrentRow) {
    return ($PreviousRow -ge 0 -and
            ($CurrentRow -lt 0 -or $PreviousRow - $CurrentRow -gt 20))
}

if (Test-RecoveryExit3End -1 -1) { throw 'RECOVERY exits before exit corner 3 has been found' }
if (-not (Test-RecoveryExit3End 35 -1)) { throw 'RECOVERY does not exit when exit corner 3 disappears' }
if (Test-RecoveryExit3End 40 20) { throw 'RECOVERY accepts an upward change of only 20 rows' }
if (-not (Test-RecoveryExit3End 40 19)) { throw 'RECOVERY rejects an upward change of 21 rows' }
if (Test-RecoveryExit3End 20 45) { throw 'RECOVERY exits when exit corner 3 moves downward' }

Write-Output 'PASS camera eight-state ring machine'
