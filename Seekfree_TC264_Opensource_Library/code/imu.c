/**
 * imu.c  ---  IMU660RB (LSM6DSR) 陀螺仪驱动实现
 *
 * 基于逐飞 zf_device_imu660rb 库:
 *   - 通过 SPI 与 IMU660RB 通信
 *   - 提供角速度和加速度读取
 *   - 陀螺仪校准 (去除零偏)
 *   - Z轴角度积分 (用于姿态估计)
 *
 * 坐标约定 (逐飞库默认):
 *   Z轴朝上, X轴朝车头方向, Y轴朝车身右侧
 */
#include "zf_common_headfile.h"
#include "zf_device_imu660rb.h"
#include "imu.h"

/* ---- 校准变量 ---- */
static float s_GyroZBias = 0.0f;     /* Z轴陀螺仪零偏 (度/s) */

/* ---- 最新IMU数据 (Update后更新) ---- */
static float s_YawRate    = 0.0f;    /* Z轴角速度 (度/s) */
static float s_RollRate   = 0.0f;    /* X轴角速度 (度/s) */
static float s_PitchRate  = 0.0f;    /* Y轴角速度 (度/s) */
static float s_AccX       = 0.0f;    /* X轴加速度 (g) */
static float s_AccY       = 0.0f;    /* Y轴加速度 (g) */
static float s_AccZ       = 0.0f;    /* Z轴加速度 (g) */

/* ---- Z轴角度积分 ---- */
static float s_AngleZ     = 0.0f;    /* 累计Z轴角度 (度) */

/********************************************************************
 * IMU_Init - 初始化IMU660RB
 * 返回: 0=成功, 非0=初始化失败
 ********************************************************************/
uint8 IMU_Init(void)
{
    uint8 ret = imu660rb_init();       /* 逐飞库硬件初始化 */
    s_GyroZBias = 0.0f;
    s_AngleZ    = 0.0f;
    return ret;
}

/********************************************************************
 * IMU_Calibrate - 陀螺仪零偏校准
 * sample_count: 采集样本数量
 * 原理: 静止时采集N个Z轴角速度样本, 取平均值作为零偏
 ********************************************************************/
void IMU_Calibrate(uint16_t sample_count)
{
    uint16_t i;
    float sum = 0.0f;

    for (i = 0; i < sample_count; i++)
    {
        imu660rb_get_gyro();           /* 触发一次陀螺仪读取 */
        system_delay_ms(2);            /* 等待数据就绪 */
        sum += imu660rb_gyro_z;        /* 累加Z轴角速度 */
    }

    s_GyroZBias = sum / (float)sample_count;
}

/********************************************************************
 * IMU_Update - 更新IMU数据
 * 读取加速度+角速度, 减去零偏后存入静态变量
 ********************************************************************/
void IMU_Update(void)
{
    imu660rb_get_acc();
    imu660rb_get_gyro();

    s_AccX      = imu660rb_acc_x;       /* X轴加速度 (g) */
    s_AccY      = imu660rb_acc_y;       /* Y轴加速度 (g) */
    s_AccZ      = imu660rb_acc_z;       /* Z轴加速度 (g) */
    s_RollRate  = imu660rb_gyro_x;      /* X轴角速度 (度/s) */
    s_PitchRate = imu660rb_gyro_y;      /* Y轴角速度 (度/s) */
    s_YawRate   = imu660rb_gyro_z - s_GyroZBias;  /* Z轴角速度 - 零偏 */
}

/********************************************************************
 * 数据读取函数 (只读, 不触发采集)
 ********************************************************************/
float IMU_GetYawRate(void)    { return s_YawRate; }
float IMU_GetRollRate(void)   { return s_RollRate; }
float IMU_GetPitchRate(void)  { return s_PitchRate; }
float IMU_GetAccX(void)       { return s_AccX; }
float IMU_GetAccY(void)       { return s_AccY; }
float IMU_GetAccZ(void)       { return s_AccZ; }

/********************************************************************
 * IMU_IntegrateZ - Z轴角度矩形积分
 * dt_s: 时间步长 (秒), 通常为控制周期 0.01s
 ********************************************************************/
void IMU_IntegrateZ(float dt_s)
{
    s_AngleZ += s_YawRate * dt_s;      /* 角速度 * 时间 = 角度增量 */
}

float IMU_GetAngleZ(void)
{
    return s_AngleZ;
}

void IMU_ClearAngleZ(void)
{
    s_AngleZ = 0.0f;
}