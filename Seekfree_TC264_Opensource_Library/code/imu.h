/**
 * imu.h  ---  IMU660RB (LSM6DSR) 陀螺仪驱动接口
 *
 * 基于逐飞 zf_device_imu660rb 库封装:
 *   - 初始化 + 校准
 *   - 角速度读取 (度/s) / 加速度读取 (g)
 *   - Z轴角度积分 (通过角速度*dt累加)
 */
#ifndef __IMU_H__
#define __IMU_H__

#include <stdint.h>

/* ---- 坐标约定 (逐飞库默认) ----
 * gyro_z : 绕Z轴角速度 (度/s, 正值=逆时针)
 * acc_x  : X轴加速度 (g, 正值=车头方向)
 * acc_y  : Y轴加速度 (g, 正值=车身右侧)
 */

/********************************************************************
 * 初始化与校准
 ********************************************************************/
uint8   IMU_Init(void);                          /* 返回0=成功, 非0=失败 */
void    IMU_Calibrate(uint16_t sample_count);    /* 采集指定样本数, 计算零偏 */

/********************************************************************
 * 数据更新与读取
 ********************************************************************/
void    IMU_Update(void);                        /* 读取最新IMU数据 */
float   IMU_GetYawRate(void);                    /* Z轴角速度 (度/s) */
float   IMU_GetRollRate(void);                   /* X轴角速度 (度/s) */
float   IMU_GetPitchRate(void);                  /* Y轴角速度 (度/s) */
float   IMU_GetAccX(void);                       /* X轴加速度 (g) */
float   IMU_GetAccY(void);                       /* Y轴加速度 (g) */
float   IMU_GetAccZ(void);                       /* Z轴加速度 (g) */

/********************************************************************
 * Z轴角度积分 (用于姿态估计)
 ********************************************************************/
void    IMU_IntegrateZ(float dt_s);              /* dt_s: 时间步长(秒) */
float   IMU_GetAngleZ(void);                     /* 返回累计Z轴角度(度) */
void    IMU_ClearAngleZ(void);                   /* 清零累计角度 */

#endif