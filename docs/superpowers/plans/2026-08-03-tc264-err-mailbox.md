# TC264 Err原子单槽邮箱实施计划

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 在不修改圆环识别和逐飞摄像头库的前提下，让CPU1只对CPU0发布的新Err执行一次舵机PD，避免10 ms控制周期重复消费约20 ms图像结果。

**Architecture:** 使用TC264 iLLD现有的`IfxCpu_acquireMutex()`/`IfxCpu_releaseMutex()`保护一个只保存最新Err的单槽邮箱。CPU0完成一帧误差计算后覆盖邮箱；CPU1保持10 ms电机控制周期，只在成功取到新Err时执行一次PD，并把局部快照传入PD。

**Tech Stack:** TC264/AURIX iLLD、ADS工程、GBK/CRLF C源码、PowerShell离散时序仿真。

## Global Constraints

- 不修改`Camera.c`、`Camera.h`中的圆环识别逻辑，也不修改逐飞`zf_device`摄像头函数。
- CPU0继续负责图像处理，CPU1继续负责电机和舵机控制。
- 不新建C或H文件；现有C/H保持GBK、无BOM、CRLF，并使用简体中文注释。
- 电机和舵机现有限幅边界保持不变。
- ADS编译和烧录由用户执行；本次只做静态检查与主机时序仿真。
- 保留用户尚未提交的`IPS200_DISPLAY_IMAGE_ENABLE`修改，不把它纳入本次提交。

---

### Task 1: 建立双核时序仿真红灯测试

**Files:**
- Create: `tests/err_mailbox_simulation_check.ps1`

**Interfaces:**
- Consumes: `Shared.h`、`PID.h`、`PID.c`、`cpu0_main.c`、`cpu1_main.c`源代码。
- Produces: 可重复执行的结构检查、双核新数据唯一消费检查和延迟对比。

- [ ] 写入对邮箱接口和时序行为的断言。
- [ ] 执行测试并确认因邮箱接口尚未实现而失败。

### Task 2: 落地TC264原子单槽邮箱

**Files:**
- Modify: `Seekfree_TC264_Opensource_Library/code/Shared.h`
- Modify: `Seekfree_TC264_Opensource_Library/code/PID.h`
- Modify: `Seekfree_TC264_Opensource_Library/code/PID.c`
- Modify: `Seekfree_TC264_Opensource_Library/user/cpu0_main.c`
- Modify: `Seekfree_TC264_Opensource_Library/user/cpu1_main.c`

**Interfaces:**
- Produces: `Shared_PublishErr(float)`、`Shared_TakeErr(float *)`、`PD_Update(float,float,float)`。

- [ ] 在`Shared.h`中基于iLLD互斥原语实现最小邮箱接口。
- [ ] CPU0按帧发布局部Err，保持EXIT2沿用上一帧的现有规则。
- [ ] CPU1只对成功取得的新Err调用PD，电机环继续每10 ms执行。
- [ ] PD改用CPU1局部Err快照并保留舵机双向限幅。
- [ ] 执行仿真、静态检查、编码与换行检查。

### Task 3: 独立Debug与提交

**Files:**
- Review: 本计划列出的全部改动文件。

**Interfaces:**
- Consumes: 完整差异、测试输出和双核模拟结果。
- Produces: 子智能体Debug结论、必要修正、最终Git提交。

- [ ] 调用子智能体检查锁竞争、重复消费、丢失更新、延迟和TC264兼容性。
- [ ] 修正Critical/Important问题并重新验证。
- [ ] 仅暂存本任务文件并提交Git。
