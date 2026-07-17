# 舵机方向与十字处理实施计划

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 修正舵机响应方向，并让已识别的十字元素通过连续边界补线保持直行。

**Architecture:** CPU0 继续负责图像元素处理和 Err 生成，CPU1 继续负责 PD 与舵机输出。十字处理在 `Camera.c` 内利用近端有效边界和远端连续三行有效边界做线性插值；PD 只修正 Err 符号，不改变增益、死区或限幅。

**Tech Stack:** TC264 C、逐飞库、PowerShell 静态回归脚本、Git。

## Global Constraints

- 所有源码与文档保持 GBK 编码且无 BOM。
- 不修改 `zf_device` 中的摄像头或陀螺仪函数。
- CPU0 负责图像处理，CPU1 负责舵机控制。
- 舵机输出保持 9 至 132 度双向限幅，中心为 80 度。
- 保留工作区中用户未提交的 `MOTOR_SPEED=20`。

---

### Task 1: 修正舵机打角方向

**Files:**
- Create: `tests/steering_direction_check.ps1`
- Modify: `Seekfree_TC264_Opensource_Library/code/PID.c:18`

**Interfaces:**
- Consumes: CPU0 发布的 `volatile float Err`，定义为 `Center - ImageSensorMid`。
- Produces: `PD_Update(float Kp, float Kd)` 中与 Err 同号的 PD 输入。

- [ ] **Step 1: 写失败回归检查**

脚本读取 GBK `PID.c`，要求包含 `s_pd_err0 = Err;`，不包含 `s_pd_err0 = -Err;`，并模拟 `Err=10` 时输出大于 80 度、`Err=-10` 时输出小于 80 度，同时检查 9 至 132 度限幅。

- [ ] **Step 2: 运行检查并确认失败**

Run: `powershell -ExecutionPolicy Bypass -File tests/steering_direction_check.ps1`

Expected: FAIL，提示 PD 仍对 Err 取反。

- [ ] **Step 3: 写最小实现**

```c
s_pd_err1 = s_pd_err0;
s_pd_err0 = Err;
```

- [ ] **Step 4: 运行全部回归检查**

Run: `Get-ChildItem tests -Filter '*.ps1' | Sort-Object Name | ForEach-Object { & $_.FullName }`

Expected: 所有脚本输出 `PASS`。

- [ ] **Step 5: 提交**

```powershell
git add tests/steering_direction_check.ps1 Seekfree_TC264_Opensource_Library/code/PID.c
git commit -m "fix: 修正舵机打角方向"
```

### Task 2: 恢复十字直行补线

**Files:**
- Create: `tests/camera_cross_handling_check.ps1`
- Modify: `Seekfree_TC264_Opensource_Library/code/Camera.c:1031-1130`

**Interfaces:**
- Consumes: `ImageStatus.WhiteLine`、`ImageStatus.OFFLine`、每行 `IsLeftFind/IsRightFind` 和边界。
- Produces: 十字区域连续的 `LeftBorder`、`RightBorder`、`Wide` 和 `Center`。

- [ ] **Step 1: 写失败回归检查**

脚本读取 GBK `Camera.c`，检查：十字分支位于 `straight_long` 与 `Bend_Road` 之前；补线验证连续三行 `T`；使用近远锚点计算插值；左右边界均经过 `LimitL/LimitH`；最后重新计算 `Wide` 和 `Center`。

- [ ] **Step 2: 运行检查并确认失败**

Run: `powershell -ExecutionPolicy Bypass -File tests/camera_cross_handling_check.ps1`

Expected: FAIL，提示现有 `Get_ExtensionLine()` 没有远端锚点插值。

- [ ] **Step 3: 写最小十字补线实现**

在 `Camera.c` 内增加静态辅助函数：

```c
static void Repair_Cross_Border(uint8 is_left);
```

函数从 `SCAN_BASE_END_ROW - 1` 向 `ImageStatus.OFFLine + 2` 扫描第一行 `W`，以其下一行为近锚点；继续向远场搜索连续三行 `T` 作为远锚点。找到远锚点时线性插值，未找到时沿用近锚点边界。所有数组访问先保证行号位于 0 至 `LCDH-1`。

- [ ] **Step 4: 调整处理优先级并重算中心**

```c
else if (ImageStatus.WhiteLine >= 8)
    Get_ExtensionLine();
else if (ImageFlag.straight_long)
    Straight_long_handle();
else if (ImageFlag.Bend_Road != 0)
    Element_Handle_Bend();
```

补线后逐行对左右边界调用 `LimitL/LimitH`。若左边界不小于右边界，则沿用下一近场行的有效边界；随后重新计算 `Wide` 和 `Center`。

- [ ] **Step 5: 运行全部检查和编码校验**

Run: `Get-ChildItem tests -Filter '*.ps1' | Sort-Object Name | ForEach-Object { & $_.FullName }`

Expected: 所有脚本输出 `PASS`。

Run: `git diff --check`

Expected: 无输出，退出码 0；所有修改文件通过 GBK 严格解码且无 BOM。

- [ ] **Step 6: 提交**

```powershell
git add tests/camera_cross_handling_check.ps1 Seekfree_TC264_Opensource_Library/code/Camera.c
git commit -m "feat: 补全十字直行处理"
```
