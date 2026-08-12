$ErrorActionPreference = 'Stop'
$Root = Split-Path -Parent $PSScriptRoot
$Gbk = [Text.Encoding]::GetEncoding(936)
$Cpu1Path = Join-Path $Root 'Seekfree_TC264_Opensource_Library/user/cpu1_main.c'
$Cpu0Path = Join-Path $Root 'Seekfree_TC264_Opensource_Library/user/cpu0_main.c'
$Cpu1 = [IO.File]::ReadAllText($Cpu1Path, $Gbk)
$Cpu0 = [IO.File]::ReadAllText($Cpu0Path, $Gbk)

$Gate = $Cpu1.IndexOf('if (!PID_Flag)')
$Take = $Cpu1.IndexOf('Shared_TakeErr(&new_position_err, &new_ring_entry_slowdown)')
$Pd = $Cpu1.IndexOf('PD_Update(PD_KP, PD_KD, position_err)')
if ($Gate -lt 0 -or $Take -lt 0 -or $Pd -lt 0) {
    throw 'Missing steering Err chain node'
}
if ($Take -gt $Gate -or $Pd -gt $Gate) {
    throw 'New Err is still delayed by the 10ms motor control gate'
}
if (([regex]::Matches($Cpu1, [regex]::Escape('PD_Update(PD_KP, PD_KD, position_err)')).Count) -ne 1) {
    throw 'Each new Err must update PD exactly once'
}
if (-not $Cpu0.Contains('#define STEERING_LOOKAHEAD_ROW 38')) {
    throw 'Steering lookahead row 38 was not preserved'
}

Write-Output 'PASS immediate steering Err response'
