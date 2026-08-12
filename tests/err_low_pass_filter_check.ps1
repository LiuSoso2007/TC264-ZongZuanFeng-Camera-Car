$ErrorActionPreference = 'Stop'

$root = Split-Path -Parent $PSScriptRoot
$cpu0Path = Join-Path $root 'Seekfree_TC264_Opensource_Library/user/cpu0_main.c'
$cpu0 = Get-Content -LiteralPath $cpu0Path -Encoding utf8 -Raw

function Assert-Contains([string]$text, [string]$expected, [string]$message)
{
    if (-not $text.Contains($expected)) { throw $message }
}

Assert-Contains $cpu0 '#define STEERING_LOOKAHEAD_ROW 41' '前瞻行不再是固定的41'
Assert-Contains $cpu0 '#define ERR_FILTER_NEW_PERCENT 70U' 'Err新值权重不再是70%'
Assert-Contains $cpu0 '#define ERR_FILTER_OLD_PERCENT 30U' 'Err旧值权重不再是30%'
Assert-Contains $cpu0 's_filtered_err = frame_err;' 'Err滤波器首帧没有直接初始化'
Assert-Contains $cpu0 'Shared_PublishErr(s_filtered_err, ring_entry_slowdown);' 'CPU0未发布滤波后的Err'

$filtered = 10.0
$filtered = (70.0 * 0.0 + 30.0 * $filtered) / 100.0
if ([Math]::Abs($filtered - 3.0) -gt 0.00001) { throw 'Err滤波模型首步不符合70/30' }
$filtered = (70.0 * 0.0 + 30.0 * $filtered) / 100.0
if ([Math]::Abs($filtered - 0.9) -gt 0.00001) { throw 'Err滤波模型连续衰减不符合70/30' }

Write-Output 'PASS Err low-pass filter: new=70%, old=30%, lookahead=41'
