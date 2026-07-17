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

Assert-Contains $Camera '#define CROSS_WHITE_LINE_MIN 8' 'Cross threshold is not defined'
Assert-Contains $Camera 'static void Repair_Cross_Border(uint8 is_left)' 'Cross border repair is missing'
Assert-Contains $Camera "row - 1].IsLeftFind == 'T'" 'Left far anchor lacks three-line validation'
Assert-Contains $Camera "row - 2].IsLeftFind == 'T'" 'Left far anchor lacks three-line validation'
Assert-Contains $Camera "row - 1].IsRightFind == 'T'" 'Right far anchor lacks three-line validation'
Assert-Contains $Camera "row - 2].IsRightFind == 'T'" 'Right far anchor lacks three-line validation'
Assert-Contains $Camera '(far_border - near_border)' 'Cross repair lacks linear interpolation'
Assert-Contains $Camera 'LimitL(ImageDeal[row].LeftBorder);' 'Cross left border is not clamped'
Assert-Contains $Camera 'LimitH(ImageDeal[row].RightBorder);' 'Cross right border is not clamped'
Assert-Contains $Camera 'ImageDeal[row].Wide = ImageDeal[row].RightBorder - ImageDeal[row].LeftBorder;' 'Cross width is not rebuilt'
Assert-Contains $Camera 'ImageDeal[row].Center = (ImageDeal[row].LeftBorder + ImageDeal[row].RightBorder) / 2;' 'Cross center is not rebuilt'

$CrossBranch = $Camera.IndexOf('else if (ImageStatus.WhiteLine >= CROSS_WHITE_LINE_MIN)')
$StraightBranch = $Camera.IndexOf('else if (ImageFlag.straight_long)', $CrossBranch)
$BendBranch = $Camera.IndexOf('else if (ImageFlag.Bend_Road != 0)', $CrossBranch)
if ($CrossBranch -lt 0 -or $StraightBranch -lt 0 -or $BendBranch -lt 0 -or
    $CrossBranch -gt $StraightBranch -or $CrossBranch -gt $BendBranch) {
    throw 'Cross handling must run before straight-long and bend handling'
}

Write-Output 'PASS camera cross handling'
