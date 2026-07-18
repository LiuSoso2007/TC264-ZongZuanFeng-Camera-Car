# IPS200 QSPI2寄存器直刷实施计划

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** CPU0使用QSPI2寄存器连续刷新188x120灰度图，消除逐字节等待，使IPS200刷新达到摄像头处理链路可用的30至40帧。

**Architecture:** 保留逐飞IPS200初始化和少量命令接口，不修改device库。项目层`IPS200.c`在设置显示区域后，将两个RGB565像素打包成一个32位QSPI数据项，直接写`MODULE_QSPI2.DATAENTRY[0]`；CPU0每帧只调用此快速接口。

**Tech Stack:** TC264、TASKING C99、QSPI2寄存器、逐飞IPS200初始化接口、PowerShell静态回归检查。

## Global Constraints

- 严禁修改CPU1。
- 严禁修改逐飞device库中的摄像头和IPS200函数。
- 修改文件使用GBK编码且无BOM，并添加简体中文注释。
- QSPI等待必须有超时保护，参数必须校验屏幕边界。
- 完成后运行全部回归检查并提交Git。

---

### Task 1: QSPI2双像素寄存器流

**Files:**
- Modify: `Seekfree_TC264_Opensource_Library/code/IPS200.c`
- Modify: `Seekfree_TC264_Opensource_Library/code/IPS200.h`
- Modify: `Seekfree_TC264_Opensource_Library/user/cpu0_main.c`
- Modify: `tests/ips200_register_stream_check.ps1`
- Modify: `tests/camera_display_rate_check.ps1`

**Interfaces:**
- Consumes: `mt9v03x_image[0]`、`MODULE_QSPI2`、逐飞SPI命令函数。
- Produces: `void IPS200_ShowGrayImageFast(const uint8 *image, uint16 width, uint16 height)`。

- [x] **Step 1: 扩展失败回归检查**

```powershell
Assert-Contains $Ips 'stream_config.B.DL = 31;' 'QSPI stream is not packing two pixels'
Assert-NotContains $Cpu0 'Camera_ShowDebug();' 'CPU0 still enters the slow display path'
```

- [x] **Step 2: 运行检查并确认失败**

Run: `powershell -ExecutionPolicy Bypass -File tests/ips200_register_stream_check.ps1`
Expected: FAIL，提示仍使用16位单像素或CPU0仍进入慢速显示路径。

- [x] **Step 3: 实现最小寄存器直刷**

```c
stream_config.B.DL = 31;
MODULE_QSPI2.DATAENTRY[0].U =
    ((uint32)ips200_gray_rgb565[image[0]] << 16)
    | ips200_gray_rgb565[image[1]];
```

CPU0在每个图像帧完成处理后仅调用：

```c
IPS200_ShowGrayImageFast(mt9v03x_image[0], MT9V03X_W, MT9V03X_H);
```

- [x] **Step 4: 验证回归、编码和保护范围**

Run: `Get-ChildItem tests -Filter *.ps1 | ForEach-Object { & $_.FullName }`
Expected: 所有检查PASS；CPU1和逐飞device库哈希不变；修改文件为GBK且无BOM。

- [x] **Step 5: 提交**

```powershell
git add -- docs/superpowers/plans/2026-07-18-ips200-qspi-register-stream.md tests/ips200_register_stream_check.ps1 tests/camera_display_rate_check.ps1 Seekfree_TC264_Opensource_Library/code/IPS200.c Seekfree_TC264_Opensource_Library/code/IPS200.h Seekfree_TC264_Opensource_Library/user/cpu0_main.c
git commit -m "perf: 使用QSPI2寄存器加速屏幕刷新"
```
