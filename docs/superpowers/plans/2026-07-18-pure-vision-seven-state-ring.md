# Pure Vision Seven-State Ring Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Replace the premature three-state ring flow with a seven-state, image-only state machine that enters, follows, exits, and releases a ring without IMU input.

**Architecture:** Keep `image_element_rings` as the locked left/right direction and use `image_element_rings_flag` as the single seven-value stage. Derive every transition from current image borders, missing-line trends, corner evidence, consecutive-frame confirmation, and bounded forward-only timeouts. Rebuild only the center line in `Camera.c`; CPU1 and all Seekfree device code stay untouched.

**Tech Stack:** TC264, TASKING C, 94x60 binary image, PowerShell regression checks, Git.

## Global Constraints

- Use seven stage values: IDLE, CONFIRM, APPROACH, ENTRY, INSIDE, EXIT, RECOVERY.
- Pure vision only; no IMU or yaw dependency.
- Do not modify CPU1, camera device code, or IMU device code.
- Keep source files GBK encoded without BOM and add Simplified Chinese comments.
- Use fixed-size state only; no allocation and no new dependency.
- Preserve unrelated user changes in `user/cpu0_main.c` and `user/cpu1_main.c`.
- Run all regression scripts, encoding checks, and the available TASKING build before committing.

---

### Task 1: Red regression for the seven-state contract

**Files:**
- Modify: `tests/camera_ring_state_check.ps1`

**Interfaces:**
- Consumes: source text from `Camera.c` and `Camera.h`.
- Produces: executable checks for all seven states, guarded transitions, timeout progress, recovery lockout, and state-specific center offsets.

- [ ] Replace the old three-state model with values 0 through 6 and assert `CONFIRM_FRAMES`, `EXIT_CONFIRM_FRAMES`, `EXIT_STABLE_FRAMES`, and `RECOVERY_FRAMES`.
- [ ] Model `CONFIRM -> APPROACH -> ENTRY -> INSIDE -> EXIT -> RECOVERY -> IDLE` and assert a transient stable frame cannot skip out of INSIDE.
- [ ] Assert INSIDE and EXIT timeouts move forward so the machine cannot remain locked forever.
- [ ] Run `powershell -ExecutionPolicy Bypass -File tests/camera_ring_state_check.ps1`; expect failure because the new constants are absent.

### Task 2: Minimal image-only implementation

**Files:**
- Modify: `Seekfree_TC264_Opensource_Library/code/Camera.h`
- Modify: `Seekfree_TC264_Opensource_Library/code/Camera.c`

**Interfaces:**
- Consumes: `ImageStatus`, `ImageDeal`, `image_element_rings`, and the existing left/right ring candidate detectors.
- Produces: seven persistent stages and a clamped center line with APPROACH, ENTRY, INSIDE, EXIT, and RECOVERY behavior.

- [ ] Add the seven stage constants and fixed frame/offset calibration constants to `Camera.h`.
- [ ] Add persistent counters, corner/exit evidence helpers, one transition helper, and forward-only timeout handling to `Camera.c`.
- [ ] Change candidate detection to enter CONFIRM and require consecutive image evidence before APPROACH.
- [ ] Apply direction-symmetric center offsets so ENTRY and INSIDE produce an error outside the existing steering dead zone.
- [ ] Require observed exit-side loss plus consecutive exit recovery before EXIT; require stable straight borders before RECOVERY and release after lockout.
- [ ] Run the ring regression; expect `PASS camera seven-state ring machine`.

### Task 3: Verification and focused commit

**Files:**
- Verify: all tracked test scripts and modified sources.

**Interfaces:**
- Consumes: Task 1 and Task 2 output.
- Produces: verified build evidence and one focused Git commit.

- [ ] Run every `tests/*.ps1` script and require all PASS.
- [ ] Verify modified text is strict GBK and has no BOM.
- [ ] Run the TASKING build available from `Seekfree_TC264_Opensource_Library/Debug` and inspect the exit code.
- [ ] Review `git diff --check` and confirm no device-library or user CPU file was included.
- [ ] Stage only the plan, ring test, `Camera.c`, and `Camera.h`; commit with `feat: ÂäµØ´¿ÊÓ¾õÆß½×¶ÎÔ²»·×´Ì¬»ú`.
