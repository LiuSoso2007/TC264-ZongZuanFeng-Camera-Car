$ErrorActionPreference = 'Stop'
$Root = Split-Path -Parent $PSScriptRoot
$Gbk = [Text.Encoding]::GetEncoding(936)

function Assert-Contains([string]$Text, [string]$Expected, [string]$Message) {
    if (-not $Text.Contains($Expected)) { throw $Message }
}

$CameraPath = Join-Path $Root 'Seekfree_TC264_Opensource_Library/code/Camera.c'
$Camera = [IO.File]::ReadAllText($CameraPath, $Gbk)

# ponytail: ??????????????????????????????
# ?????? + BlackHole_Check_Bottom ??????????????
Assert-Contains $Camera `
    'ImageStatus.Miss_Left_lines > 10' `
    'Left ring detection lacks opposite-side loss guard'
Assert-Contains $Camera `
    'ImageStatus.Miss_Right_lines > 10' `
    'Right ring detection lacks opposite-side loss guard'
Assert-Contains $Camera `
    'for (row = SCAN_BASE_START_ROW; (row - 2) > ImageStatus.OFFLine; row -= 2)' `
    'Center-line drawing can still connect an invalid OFFLine row'

# ponytail: ?????>=6???<=5???
function Test-RingCandidate([int]$SameSideMiss, [int]$OtherSideMiss, [int]$JumpCount) {
    return $SameSideMiss -ge 4 -and $OtherSideMiss -le 10 -and $JumpCount -ge 1
}

if (Test-RingCandidate -SameSideMiss 0 -OtherSideMiss 0 -JumpCount 1) {
    throw 'Straight-road noise is still accepted as a ring candidate'
}
if (-not (Test-RingCandidate -SameSideMiss 4 -OtherSideMiss 0 -JumpCount 1)) {
    throw 'A valid ring candidate is rejected by the regression model'
}

Write-Output 'PASS camera straight guard'
