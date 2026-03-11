/**
 ******************************************************************************
 * @file           : crt_gimbal.cpp
 * @brief          : 云台控制
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2025 GMaster
 * All rights reserved.
 *
 ******************************************************************************
 */
/* Includes ------------------------------------------------------------------*/
#include "crt_gimbal.hpp"
#include "para_gimbal.hpp"
#include "tsk_isr.hpp"
#include "drv_misc.h"
#include <math.h>

/* Typedef -------------------------------------------------------------------*/

/* Define --------------------------------------------------------------------*/

/* Macro ---------------------------------------------------------------------*/

/* Variables -----------------------------------------------------------------*/

/* Function prototypes -------------------------------------------------------*/

/* User code -----------------------------------------------------------------*/
/******************************************************************************
 *                            Gimbal类实现
 ******************************************************************************/

extern Vofa<12> vofa;

Gimbal::Gimbal(MotorDM4310 *yawMotor, MotorDM4310 *pitchMotor, IMU *imu)
    : m_yawMotor(yawMotor), m_pitchMotor(pitchMotor),
      m_imu(imu),
      m_gimbalMode(GIMBAL_NO_FORCE),
      m_yawTargetAngle(0.0f), m_pitchTargetAngle(0.0f),
      m_remoteControl(),
      m_isInitComplete(false) {}

void Gimbal::init()
{
    DWT_Init();
    CAN_Init(&hcan1, can1RxCallback);
    CAN_Init(&hcan2, can2RxCallback);
    UART_Init(&huart3, dr16RxCallback, 36);
    m_imu->init();
    m_isInitComplete = true;
}

void Gimbal::controlLoop()
{
    if (!m_isInitComplete) return;
    vofa.writeData(m_gimbalMode);
    modeSelect();
    targetOrientationPlan();
    pitchControl();
    yawControl();
    transmitGimbalMotorData();
}

void Gimbal::targetOrientationPlan()
{
    switch (m_gimbalMode) {
        case MANUAL_CONTROL:
            // setYawAngle(m_yawTargetAngle - rcStickDeadZoneFilter(m_remoteControl.getRightStickX()) * DT7_STICK_YAW_SENSITIVITY*0.1f);//0.6为遥控器灵敏度精度经检查无法精确到小数点后三位，故再加神秘小常数
            // setPitchAngle(m_pitchTargetAngle - rcStickDeadZoneFilter(m_remoteControl.getRightStickY()) * DT7_STICK_PITCH_SENSITIVITY*0.1f);//0.6为遥控器灵敏度精度经检查无法精确到小数点后三位，故再加神秘小常数
             setYawAngle( -rcStickDeadZoneFilter(m_remoteControl.getRightStickX()) * YAW_UPPER_LIMIT);
             setPitchAngle( -rcStickDeadZoneFilter(m_remoteControl.getRightStickY()) * PITCH_UPPER_LIMIT);
            break;

        case AUTO_CONTROL:
            break;

        default:
            break;
    }
}


void Gimbal::imuLoop()
{
    if (!m_isInitComplete) return;
    m_eulerAngle = m_imu->solveAttitude();
}

void Gimbal::receiveGimbalMotorDataFromISR(const can_rx_message_t *rxMessage)
{
    if (m_yawMotor->decodeCanRxMessageFromISR(rxMessage)) return;
    if (m_pitchMotor->decodeCanRxMessageFromISR(rxMessage)) return;
}

void Gimbal::receiveRemoteControlDataFromISR(const uint8_t *rxData)
{
    m_remoteControl.receiveRxDataFromISR(rxData);
}

void Gimbal::modeSelect()
{
    m_remoteControl.updateEvent();
    if (!m_remoteControl.isConnected()) {
        m_gimbalMode  = GIMBAL_NO_FORCE;
        return;
    }

    switch (m_remoteControl.getRightSwitchStatus()) {
        case Dr16RemoteControl::SwitchStatus3Pos::SWITCH_DOWN:
            m_gimbalMode  = GIMBAL_NO_FORCE;
            if (m_remoteControl.getLeftSwitchEvent() == Dr16RemoteControl::SwitchEvent3Pos::SWITCH_TOGGLE_MIDDLE_UP) {
                m_gimbalMode = CALIBRATION;
            }
            break;

        case Dr16RemoteControl::SwitchStatus3Pos::SWITCH_MIDDLE:
            m_gimbalMode  = MANUAL_CONTROL;
            break;

        case Dr16RemoteControl::SwitchStatus3Pos::SWITCH_UP:
            m_gimbalMode  = AUTO_CONTROL;
            break;

        default:
            break;
    }

    switch (m_remoteControl.getLeftSwitchStatus()) {
        case Dr16RemoteControl::SwitchStatus3Pos::SWITCH_DOWN:
            break;

        case Dr16RemoteControl::SwitchStatus3Pos::SWITCH_MIDDLE:
            break;

        case Dr16RemoteControl::SwitchStatus3Pos::SWITCH_UP:
            m_gimbalMode = GIMBAL_LOCK;
            break;

        default:
            break;
    }
}

void Gimbal::pitchControl()
{
    switch (m_gimbalMode) {
        case GIMBAL_NO_FORCE:
            m_pitchMotor->openloopControl(0.0f);
            break;

        case CALIBRATION:
            break;

        case MANUAL_CONTROL:
        case AUTO_CONTROL: { // 手动控制和自动控制都使用同样的闭环控制
            fp32 fdbData[2] = {(m_pitchTargetAngle - m_pitchMotor->getCurrentAngle()), -m_pitchMotor->getCurrentAngularVelocity()};
            vofa.writeData(m_pitchMotor->getCurrentAngle());
            vofa.writeData(m_pitchTargetAngle);

            fp32 pidOutput  = m_pitchMotor->externalClosedloopControl(-3.22f, fdbData, 2);

            
/*#ifdef PITCH_GRAVITY_COMPENSATE
            fp32 totalTorque = gravityCompensate(pidOutput, m_pitchMotor->getCurrentAngle(), PITCH_GRAVITY_COMPENSATE);
            m_pitchMotor->openloopControl(totalTorque);
#else
            (void)pidOutput; // 防止未使用变量警告
#endif
            break;*/
        }

        default:
            break;
    }
}

void Gimbal::yawControl()
{
    switch (m_gimbalMode) {
        case GIMBAL_NO_FORCE:
            m_yawTargetAngle = m_eulerAngle.z;
            m_yawMotor->openloopControl(0.0f);
            break;

        case CALIBRATION:
            // m_yawMotor->setMotorZeroPosition();
            break;

        case MANUAL_CONTROL:
        case AUTO_CONTROL: { // 手动控制和自动控制都使用同样的闭环控制
            fp32 fdbData[2] = {m_eulerAngle.z - m_yawTargetAngle, m_imu->getGyro().z};
           fp32 pidOutput = m_yawMotor->externalClosedloopControl(0.0f, fdbData, 2);

            break;
            
        }

        default:
            break;
    }
}

void Gimbal::transmitGimbalMotorData()
{
    HAL_CAN_AddTxMessage(&hcan1, m_yawMotor->getMotorControlHeader(), m_yawMotor->getMotorControlData(), NULL);
    HAL_CAN_AddTxMessage(&hcan1, m_pitchMotor->getMotorControlHeader(), m_pitchMotor->getMotorControlData(), NULL);
}

inline void Gimbal::setPitchAngle(const fp32 &targetAngle)
{
    if (targetAngle > PITCH_UPPER_LIMIT)
        m_pitchTargetAngle = PITCH_UPPER_LIMIT;
    else if (targetAngle < PITCH_LOWER_LIMIT)
        m_pitchTargetAngle = PITCH_LOWER_LIMIT;
    else
        m_pitchTargetAngle = targetAngle;
}

inline void Gimbal::setYawAngle(const fp32 &targetAngle)
{
    if (targetAngle > YAW_UPPER_LIMIT)
        m_yawTargetAngle = YAW_UPPER_LIMIT;
    else if (targetAngle < YAW_LOWER_LIMIT)
        m_yawTargetAngle = YAW_LOWER_LIMIT;
    else
        m_yawTargetAngle = targetAngle;
    // 借用normalizeDeltaAngle函数将目标角度限制在-PI到PI之间
    // m_yawTargetAngle = GSRLMath::normalizeDeltaAngle(targetAngle);
}

inline void Gimbal::convertGimbalTargetSpeedToChassisTargetSpeed()
{
}

inline fp32 Gimbal::rcStickDeadZoneFilter(const fp32 &rcStickValue)
{
    if (rcStickValue > DT7_STICK_DEAD_ZONE)
        return (rcStickValue - DT7_STICK_DEAD_ZONE) / (1.0f - DT7_STICK_DEAD_ZONE);
    else if (rcStickValue < -DT7_STICK_DEAD_ZONE)
        return (rcStickValue + DT7_STICK_DEAD_ZONE) / (1.0f - DT7_STICK_DEAD_ZONE);
    else
        return 0.0f;
}

/**
 * @brief 重力前馈补偿计算
 * @param baseTorque 基础力矩(PID输出)
 * @param currentAngle 当前角度(rad)
 * @param compensateCoeff 补偿系数(Nm)
 * @return 叠加前馈后的总力矩
 */
inline fp32 Gimbal::gravityCompensate(fp32 baseTorque, fp32 currentAngle, fp32 compensateCoeff)
{
    return baseTorque + compensateCoeff * cos(currentAngle);
}
