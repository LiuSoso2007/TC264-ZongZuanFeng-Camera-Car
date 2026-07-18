# 圆环状态机实施计划

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 让圆环方向和处理阶段跨帧保持，可靠完成入环、环内补线、出环释放，并阻止同一圆环重复进入。

**Architecture:** 保留 `image_element_rings` 表示左右方向，将 `image_element_rings_flag` 明确为单向阶段状态。`Flag_init()` 只清理逐帧元素，不再清理圆环状态；圆环处理按图像丢线状态从入环推进到环内，再以连续稳定直道帧完成出环。

**Tech Stack:** TC264、TASKING C99、PowerShell 静态回归脚本、Git。

## Global Constraints

- 只修改 CPU0 图像算法，严禁修改 CPU1。
- 严禁修改逐飞 device 库中的摄像头和陀螺仪函数。
- 所有文本文件使用 GBK 编码且不含 BOM，并添加简体中文注释。
- 保留舵机和电机现有限幅逻辑。
- 修改完成后运行全部测试并提交 Git。

---

### Task 1: 圆环跨帧状态机

**Files:**
- Create: `tests/camera_ring_state_check.ps1`
- Modify: `Seekfree_TC264_Opensource_Library/code/Camera.h`
- Modify: `Seekfree_TC264_Opensource_Library/code/Camera.c`

**Interfaces:**
- Consumes: `ImageStatus.OFFLine`、`Miss_Left_lines`、`Miss_Right_lines` 和现有左右圆环候选检测。
- Produces: `RING_STATE_IDLE/ENTRY/INSIDE/EXIT`、持久化的 `image_element_rings` 和 `image_element_rings_flag`。

- [ ] **Step 1: 写失败回归测试**

在 `tests/camera_ring_state_check.ps1` 中检查四个阶段常量、`Ring_State_Update()`、`Flag_init()` 不清零圆环状态，以及以下状态序列：

```text
ENTRY --OFFLine>=5--> INSIDE
INSIDE --双边恢复且OFFLine<=2--> EXIT
EXIT --连续8帧稳定直道--> IDLE
EXIT --再次出现圆环候选--> 仍为EXIT
```

- [ ] **Step 2: 运行测试确认失败**

Run: `powershell -ExecutionPolicy Bypass -File tests/camera_ring_state_check.ps1`

Expected: FAIL，提示缺少圆环阶段常量或 `Flag_init()` 仍清零圆环状态。

- [ ] **Step 3: 实现最小状态机**

在 `Camera.h` 定义：

```c
#define RING_STATE_IDLE    0
#define RING_STATE_ENTRY   1
#define RING_STATE_INSIDE  2
#define RING_STATE_EXIT    3
#define RING_EXIT_STABLE_FRAMES 8U
```

检测到左/右圆环时只从空闲态进入 `RING_STATE_ENTRY`。在 `Camera.c` 增加一个静态稳定帧计数和 `Ring_State_Update()`：`OFFLine>=5` 进入环内；双边恢复后进入出环态；连续8帧稳定后同时清理方向、阶段和计数。左右补线仅在 `ENTRY` 与 `INSIDE` 阶段执行。

- [ ] **Step 4: 运行完整验证**

Run: `Get-ChildItem tests -Filter *.ps1 | Sort-Object Name | ForEach-Object { & $_.FullName }`

Expected: 所有脚本输出 `PASS`。随后校验 `Camera.c`、`Camera.h` 和测试文件均为 GBK 且无 BOM，并尝试在 `Debug` 目录运行 TASKING `amk -j32 all`。

- [ ] **Step 5: 精确提交**

只暂存计划、圆环测试、`Camera.c` 和 `Camera.h`，排除 CPU1 既有工作区改动。

```text
git commit -m "fix: 使用状态机完成圆环入环与出环"
```
