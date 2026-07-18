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

Assert-Contains $Header '#define RING_STATE_IDLE    0' 'Ring idle state is missing'
Assert-Contains $Header '#define RING_STATE_ENTRY   1' 'Ring entry state is missing'
Assert-Contains $Header '#define RING_STATE_INSIDE  2' 'Ring inside state is missing'
Assert-Contains $Header '#define RING_STATE_EXIT    3' 'Ring exit state is missing'
Assert-Contains $Header '#define RING_EXIT_STABLE_FRAMES 8U' 'Ring exit debounce is missing'

Assert-Contains $Camera 'static uint8 s_ring_exit_stable_count = 0U;' 'Ring exit counter is not persistent'
Assert-Contains $Camera 'ImageFlag.image_element_rings_flag = RING_STATE_ENTRY;' 'Ring detection does not enter the state machine'
Assert-Contains $Camera 'if (ImageFlag.image_element_rings_flag == RING_STATE_ENTRY)' 'Ring entry transition is missing'
Assert-Contains $Camera 'else if (ImageFlag.image_element_rings_flag == RING_STATE_INSIDE)' 'Ring inside transition is missing'
Assert-Contains $Camera 'else if (ImageFlag.image_element_rings_flag == RING_STATE_EXIT)' 'Ring exit transition is missing'
Assert-Contains $Camera 's_ring_exit_stable_count >= RING_EXIT_STABLE_FRAMES' 'Ring exit release condition is missing'

$FlagStart = $Camera.IndexOf('void Flag_init(void)')
$FlagEnd = $Camera.IndexOf('void Camera_ShowElementStatus(void)', $FlagStart)
if ($FlagStart -lt 0 -or $FlagEnd -le $FlagStart) { throw 'Flag_init function block is missing' }
$FlagCode = $Camera.Substring($FlagStart, $FlagEnd - $FlagStart)
if ($FlagCode.Contains('ImageFlag.image_element_rings') -or
    $FlagCode.Contains('ImageFlag.ring_big_small')) {
    throw 'Flag_init still clears persistent ring state every frame'
}

function Step-RingState([int]$State, [int]$OffLine, [int]$LeftMiss,
                        [int]$RightMiss, [int]$StableCount) {
    $Stable = $OffLine -le 2 -and $LeftMiss -lt 4 -and $RightMiss -lt 4
    if ($State -eq 1 -and $OffLine -ge 5) {
        $State = 2
    }
    elseif ($State -eq 2 -and $Stable) {
        $State = 3
        $StableCount = 0
    }
    elseif ($State -eq 3) {
        if ($Stable) {
            $StableCount++
            if ($StableCount -ge 8) {
                $State = 0
                $StableCount = 0
            }
        }
        else {
            $StableCount = 0
        }
    }
    return [pscustomobject]@{ State = $State; StableCount = $StableCount }
}

function Try-EnterRing([int]$State, [bool]$Candidate) {
    if ($State -eq 0 -and $Candidate) { return 1 }
    return $State
}

$Step = Step-RingState -State 1 -OffLine 5 -LeftMiss 13 -RightMiss 0 -StableCount 0
if ($Step.State -ne 2) { throw 'Entry does not advance to inside state' }

$Step = Step-RingState -State $Step.State -OffLine 2 -LeftMiss 0 -RightMiss 0 -StableCount 0
if ($Step.State -ne 3) { throw 'Inside does not advance to exit state' }

if ((Try-EnterRing -State $Step.State -Candidate $true) -ne 3) {
    throw 'An active ring can be entered repeatedly'
}

$Step = Step-RingState -State $Step.State -OffLine 2 -LeftMiss 13 -RightMiss 0 -StableCount 4
if ($Step.State -ne 3 -or $Step.StableCount -ne 0) {
    throw 'Unstable exit does not keep the lockout state'
}

for ($Frame = 1; $Frame -le 8; $Frame++) {
    $Step = Step-RingState -State $Step.State -OffLine 2 -LeftMiss 0 -RightMiss 0 `
        -StableCount $Step.StableCount
}
if ($Step.State -ne 0) { throw 'Stable exit does not release the ring state' }

Write-Output 'PASS camera ring state machine'
