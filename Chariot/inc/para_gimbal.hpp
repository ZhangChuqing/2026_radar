/**
 ******************************************************************************
 * @file           : para_gimbal.hpp
 * @brief          : 云台参数宏定义，方便修改调参
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2025 GMaster
 * All rights reserved.
 *
 ******************************************************************************
 */

/* Define to prevent recursive inclusion -------------------------------------*/
#pragma once

/* Includes ------------------------------------------------------------------*/
#include "math_const.h"

/******************************************************************************
 *                            PID参数
 ******************************************************************************/
// 云台Yaw电机 (DM4310)
/*#define YAW_OUTER_KP 0.5f  
#define YAW_OUTER_KI 0.0f
#define YAW_OUTER_KD 0.01f
#define YAW_OUTER_OUT_LIMIT 2.0f
#define YAW_OUTER_IOUT_LIMIT 5.0f
#define YAW_INNER_KP 0.9f 
#define YAW_INNER_KI 0.0f
#define YAW_INNER_KD 0.3f
#define YAW_INNER_OUT_LIMIT 5.0f
#define YAW_INNER_IOUT_LIMIT 0.0f
#define YAW_INNER_LOWPASS_FILTER_PARA 0.6f*/
#define YAW_OUTER_KP 0.1f  
#define YAW_OUTER_KI 0.0f
#define YAW_OUTER_KD 0.0f
#define YAW_OUTER_OUT_LIMIT 10.0f
#define YAW_OUTER_IOUT_LIMIT 5.0f
#define YAW_INNER_KP 1.0f 
#define YAW_INNER_KI 0.0f
#define YAW_INNER_KD 0.525f
#define YAW_INNER_OUT_LIMIT 5.0f
#define YAW_INNER_IOUT_LIMIT 0.0f
#define YAW_INNER_LOWPASS_FILTER_PARA 1.0f
// 云台Pitch电机
#define PITCH_OUTER_KP                  1.75f // 外环
#define PITCH_OUTER_KI                  0.0f
#define PITCH_OUTER_KD                  0.0f
#define PITCH_OUTER_OUT_LIMIT           1.0f
#define PITCH_OUTER_IOUT_LIMIT          0.0f
#define PITCH_INNER_KP                  1.48f // 内环
#define PITCH_INNER_KI                  0.0f
#define PITCH_INNER_KD                  0.15f
#define PITCH_INNER_OUT_LIMIT           10.0f
#define PITCH_INNER_IOUT_LIMIT          0.0f
#define PITCH_INNER_LOWPASS_FILTER_PARA 1.0f
// 重力补偿前馈（Nm）
#define PITCH_GRAVITY_COMPENSATE 0.0f

/*****************************************************************************
 *                            IMU参数
 ******************************************************************************/
// Mahony算法参数
#define AHRS_AUTO_FREQ      0
#define AHRS_DEFAULT_FILTER 0
#define MAHONY_KP           1.0f
#define MAHONY_KI           0.0f
// IMU零飘补偿
#define GYRO_OFFSET_X  0.0f
#define GYRO_OFFSET_Y  0.0f
#define GYRO_OFFSET_Z  0.0f
#define ACCEL_OFFSET_X 0.0f
#define ACCEL_OFFSET_Y 0.0f
#define ACCEL_OFFSET_Z 0.0f
#define MAG_OFFSET_X   0.0f
#define MAG_OFFSET_Y   0.0f
#define MAG_OFFSET_Z   0.0f
// 安装朝向修正旋转矩阵
#define INSTALL_SPIN_MATRIX GSRLMath::Matrix33f((fp32[3][3]){{1, 0, 0}, {0, -1, 0}, {0, 0, -1}})
// #define INSTALL_SPIN_MATRIX GSRLMath::Matrix33f::MatrixType::IDENTITY

/******************************************************************************
 *                            遥控器灵敏度与死区
 ******************************************************************************/
#define DT7_STICK_DEAD_ZONE         0.00f
#define DT7_STICK_PITCH_SENSITIVITY 0.01f
#define DT7_STICK_YAW_SENSITIVITY   0.02f

/******************************************************************************
 *                            云台角度限制
 ******************************************************************************/
#define PITCH_UPPER_LIMIT 0.5f 
#define PITCH_LOWER_LIMIT -0.5f 
#define YAW_UPPER_LIMIT   10.0f
#define YAW_LOWER_LIMIT   -10.0f