# 直道蓝线与舵机左偏修复 Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 消除直道蓝线顶部的无效横向跳变，并让舵机按像素偏差围绕已校准的 80 度中位双向纠偏。

**Architecture:** 保留现有 CPU0 图像处理和 CPU1 运动控制边界。CPU0 只输出像素单位的 `Err`，CPU1 的现有 PD 参数继续按像素使用；舵机中位和角度上下限由 `Servo.h` 提供唯一常量。蓝线显示仅消费 `OFFLine` 以上的有效中心数据。

**Tech Stack:** TC264 C、逐飞开源库、Infineon ADS、PowerShell 回归检查、GBK 编码。

## Global Constraints

- 不修改逐飞 `libraries/zf_device` 中的摄像头和陀螺仪函数。
- 所有新增或修改文件使用 GBK 编码且不得包含 BOM。
- C 代码新增注释使用简体中文。
- 保留电机速度、`Kp=0.8`、`Kd=0.4` 和现有转向正负方向。
- 舵机中位为 80 度，最终输出范围为 9 至 132 度，必须保留中位两侧输出。
- 不覆盖 `user/cpu1_main.c` 中用户尚未提交的电机速度改动。

---

### Task 1: 建立转向链路回归检查

**Files:**
- Create: `tests/steering_contract_check.ps1`
- Test: `tests/steering_contract_check.ps1`

**Interfaces:**
- Consumes: `Camera.c` 的蓝线循环、`cpu0_main.c` 的 `Err`、`Servo.h` 的舵机常量、`PID.c` 的 PD 输出。
- Produces: 一个无外部依赖、失败时返回非零退出码的静态与数值契约检查。

- [ ] **Step 1: 写入失败的回归检查**

```powershell
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
$Cpu0 = Read-Gbk 'Seekfree_TC264_Opensource_Library/user/cpu0_main.c'
$ServoH = Read-Gbk 'Seekfree_TC264_Opensource_Library/code/Servo.h'
$ServoC = Read-Gbk 'Seekfree_TC264_Opensource_Library/code/Servo.c'
$Pid = Read-Gbk 'Seekfree_TC264_Opensource_Library/code/PID.c'

Assert-Contains $Camera 'for (row = SCAN_BASE_START_ROW; (row - 2) > ImageStatus.OFFLine; row -= 2)' '蓝线仍可能连接OFFLine无效行'
Assert-NotContains $Cpu0 '/ (float)ImageSensorMid;' 'Err仍是归一化单位'
Assert-Contains $Cpu0 'Err = 0.0f;' '无有效赛道时未清零Err'
Assert-Contains $ServoH '#define SERVO_CENTER_ANGLE  80U' '舵机中位未统一为80度'
Assert-Contains $ServoH '#define SERVO_MIN_ANGLE      9U' '缺少舵机负方向限幅'
Assert-Contains $ServoH '#define SERVO_MAX_ANGLE    132U' '缺少舵机正方向限幅'
Assert-Contains $ServoC 'SERVO_CENTER_ANGLE' '舵机初始化未使用统一中位'
Assert-Contains $Pid '+ (float)SERVO_CENTER_ANGLE;' 'PD未围绕统一中位输出'
Assert-Contains $Pid 'Servo_SetAngleDeg(SERVO_CENTER_ANGLE);' 'PD死区未使用统一中位'

function Get-PdAngle([float]$Err) {
    $Center = 80.0
    if ($Err -ge -5.0 -and $Err -le 5.0) { return $Center }
    $Out = 0.8 * (-$Err) + 0.4 * (-$Err) + $Center
    return [Math]::Max(9.0, [Math]::Min(132.0, $Out))
}

$Negative = Get-PdAngle -Err (-10.0)
$Zero = Get-PdAngle -Err 0.0
$Positive = Get-PdAngle -Err 10.0
if (-not ($Negative -gt 80.0 -and $Zero -eq 80.0 -and $Positive -lt 80.0)) {
    throw 'PD未在中位两侧产生相反方向输出'
}
if ($Negative -gt 132.0 -or $Positive -lt 9.0) { throw 'PD输出越界' }

Write-Output 'PASS steering contract'
```

- [ ] **Step 2: 运行检查并确认 RED**

Run: `powershell -ExecutionPolicy Bypass -File tests/steering_contract_check.ps1`

Expected: FAIL，首个错误为“蓝线仍可能连接OFFLine无效行”。

- [ ] **Step 3: 提交失败检查**

```bash
git add tests/steering_contract_check.ps1
git commit -m "test: 添加转向链路回归检查"
```

### Task 2: 修复有效行、误差单位和舵机输出契约

**Files:**
- Modify: `Seekfree_TC264_Opensource_Library/code/Camera.c:105`
- Modify: `Seekfree_TC264_Opensource_Library/user/cpu0_main.c:54`
- Modify: `Seekfree_TC264_Opensource_Library/code/Servo.h:4`
- Modify: `Seekfree_TC264_Opensource_Library/code/Servo.c:13`
- Modify: `Seekfree_TC264_Opensource_Library/code/PID.c:11`
- Test: `tests/steering_contract_check.ps1`

**Interfaces:**
- Consumes: `ImageStatus.OFFLine`、`ImageDeal[].Center`、CPU0/CPU1 共享变量 `Err`。
- Produces: 像素单位 `Err`，以及围绕 `SERVO_CENTER_ANGLE` 且限制在 `[SERVO_MIN_ANGLE, SERVO_MAX_ANGLE]` 的舵机角度。

- [ ] **Step 1: 修复蓝线端点范围**

将 `Camera_DrawCenterLines()` 的循环改为：

```c
    /* 两个端点都必须位于OFFLine以上的有效搜线区域。 */
    for (row = SCAN_BASE_START_ROW; (row - 2) > ImageStatus.OFFLine; row -= 2)
```

- [ ] **Step 2: 恢复像素误差并处理无效赛道**

将 CPU0 的误差计算改为：

```c
            if (ImageStatus.OFFLine < 48)
            {
                Err = (float)((ImageDeal[52].Center + ImageDeal[51].Center
                             + ImageDeal[50].Center) / 3 - ImageSensorMid);
            }
            else if (ImageStatus.OFFLine < SCAN_BASE_START_ROW
                  && ImageDeal[ImageStatus.OFFLine + 1].Wide > 8)
            {
                Err = (float)(ImageDeal[ImageStatus.OFFLine + 1].Center
                            - ImageSensorMid);
            }
            else
            {
                Err = 0.0f;
            }
```

- [ ] **Step 3: 统一舵机校准常量和双向限幅**

在 `Servo.h` 增加：

```c
/* 实车舵机校准参数，初始化和控制必须共用。 */
#define SERVO_CENTER_ANGLE  80U
#define SERVO_MIN_ANGLE      9U
#define SERVO_MAX_ANGLE    132U
```

`Servo_Init()` 使用 `SERVO_CENTER_ANGLE` 计算初始占空比；`Servo_SetAngleDeg()` 同时执行最小角和最大角限制。

- [ ] **Step 4: 让 PD 围绕统一中位输出**

将 `PID.c` 的输出和限幅改为：

```c
    s_pd_out  = Kp * s_pd_err0 + Kd * (s_pd_err0 - s_pd_err1)
              + (float)SERVO_CENTER_ANGLE;
    if (s_pd_out > (float)SERVO_MAX_ANGLE) s_pd_out = (float)SERVO_MAX_ANGLE;
    if (s_pd_out < (float)SERVO_MIN_ANGLE) s_pd_out = (float)SERVO_MIN_ANGLE;

    if (Err >= -5.0f && Err <= 5.0f)
        Servo_SetAngleDeg(SERVO_CENTER_ANGLE);
```

- [ ] **Step 5: 运行检查并确认 GREEN**

Run: `powershell -ExecutionPolicy Bypass -File tests/steering_contract_check.ps1`

Expected: `PASS steering contract`，退出码 0。

- [ ] **Step 6: 检查编码、BOM和差异**

Run: 对五个 C/H 文件及测试脚本执行严格 GBK 解码和 UTF-8 BOM 字节检查。

Expected: 所有文件可按代码页 936 解码，且文件头不是 `EF BB BF`。

Run: `git diff --check`

Expected: 退出码 0，无空白错误。

- [ ] **Step 7: 尝试工程构建**

Run: `Get-Command cctc,make,gmake -ErrorAction SilentlyContinue`

Expected: 若发现 ADS 工具，使用 `Seekfree_TC264_Opensource_Library/Debug/makefile` 构建；若均不存在，记录本机未能执行固件编译。

- [ ] **Step 8: 提交根因修复**

```bash
git add Seekfree_TC264_Opensource_Library/code/Camera.c \
        Seekfree_TC264_Opensource_Library/user/cpu0_main.c \
        Seekfree_TC264_Opensource_Library/code/Servo.h \
        Seekfree_TC264_Opensource_Library/code/Servo.c \
        Seekfree_TC264_Opensource_Library/code/PID.c
git commit -m "fix: 修复直道蓝线与舵机左偏"
```
