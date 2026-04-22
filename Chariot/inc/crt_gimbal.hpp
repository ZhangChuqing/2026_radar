/**
 ******************************************************************************
 * @file           : crt_gimbal.hpp
 * @brief          : header file for crt_gimbal.cpp
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2026 GMaster
 * All rights reserved.
 *
 ******************************************************************************
 */

/* Define to prevent recursive inclusion -------------------------------------*/
#pragma once

/* Includes ------------------------------------------------------------------*/
#include "GSRL.hpp"
#include "para_gimbal.hpp"
#include "dvc_vofa.hpp"

/* Exported types ------------------------------------------------------------*/

class Gimbal
{
public:
    using Vector3f  = GSRLMath::Vector3f;
    using Matrix33f = GSRLMath::Matrix33f;
    // 云台模式
    enum GimbalMode : uint8_t {
        GIMBAL_NO_FORCE = 0,
        CALIBRATION,
        MANUAL_CONTROL,
        AUTO_CONTROL,
        GIMBAL_LOCK
    };

private:
    // 电机
    MotorDM4310 *m_yawMotor;
    MotorDM4310 *m_pitchMotor;

    // IMU
    IMU *m_imu;
    Vector3f m_eulerAngle;

    // 云台控制相关量
    GimbalMode m_gimbalMode;
    fp32 m_yawTargetAngle;
    fp32 m_pitchTargetAngle;

    // 遥控器
    DR16RemoteControl m_remoteControl;

    // 标志位
    bool m_isInitComplete;

public:
    Gimbal(MotorDM4310 *yawMotor, MotorDM4310 *pitchMotor, IMU *imu);
    void init();
    void controlLoop();
    void targetOrientationPlan();
    void imuLoop();
    void receiveGimbalMotorDataFromISR(const can_rx_message_t *rxMessage);
    void receiveRemoteControlDataFromISR(const uint8_t *rxData);

private:
    void modeSelect();
    void pitchControl();
    void yawControl();
    void transmitGimbalMotorData();

    inline void setPitchAngle(const fp32 &targetAngle);
    inline void setYawAngle(const fp32 &targetAngle);
    inline void convertGimbalTargetSpeedToChassisTargetSpeed();
    inline fp32 rcStickDeadZoneFilter(const fp32 &rcStickValue);
    inline fp32 gravityCompensate(fp32 baseTorque, fp32 currentAngle, fp32 compensateCoeff);
};

/* Exported constants --------------------------------------------------------*/

/* Exported macro ------------------------------------------------------------*/

/* Exported functions prototypes ---------------------------------------------*/

/* Defines -------------------------------------------------------------------*/
