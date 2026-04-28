/**
 ******************************************************************************
 * @file           : crt_gimbal.cpp
 * @brief          : 云台控制
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2026 GMaster
 * All rights reserved.
 *
 ******************************************************************************
 */
/* Includes ------------------------------------------------------------------*/
#include "crt_gimbal.hpp"
#include "para_gimbal.hpp"
#include "tsk_isr.hpp"
#include "drv_misc.h"
#include "usbd_cdc_if.h"
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
      m_yawTargetAngle(YAW_CENTER_ANGLE), m_pitchTargetAngle(PITCH_CENTER_ANGLE),
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
    transmitGimbalMotorDataViaUsb();
}

void Gimbal::targetOrientationPlan()
{
    switch (m_gimbalMode) {
        case MANUAL_CONTROL:
            setYawAngle(m_yawTargetAngle - rcStickDeadZoneFilter(m_remoteControl.getRightStickX()) * YAW_LEVAL_SENSITIVITY);
            setPitchAngle(m_pitchTargetAngle - rcStickDeadZoneFilter(m_remoteControl.getRightStickY()) * PITCH_LEVAL_SENSITIVITY);
            break;

        case AUTO_CONTROL:
            if (usbRxMsg.found) {
                setYawAngle(usbRxMsg.yaw);
                setPitchAngle(usbRxMsg.pitch);
            }
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
        m_gimbalMode = GIMBAL_NO_FORCE;
        return;
    }

    switch (m_remoteControl.getRightSwitchStatus()) {
        case DR16RemoteControl::SwitchStatus3Pos::SWITCH_DOWN:
            m_gimbalMode = GIMBAL_NO_FORCE;
            if (m_remoteControl.getLeftSwitchEvent() == DR16RemoteControl::SwitchEvent3Pos::SWITCH_TOGGLE_MIDDLE_UP) {
                m_gimbalMode = CALIBRATION;
            }
            break;

        case DR16RemoteControl::SwitchStatus3Pos::SWITCH_MIDDLE:
            m_gimbalMode = MANUAL_CONTROL;
            break;

        case DR16RemoteControl::SwitchStatus3Pos::SWITCH_UP:
            m_gimbalMode = AUTO_CONTROL;
            break;

        default:
            break;
    }

    switch (m_remoteControl.getLeftSwitchStatus()) {
        case DR16RemoteControl::SwitchStatus3Pos::SWITCH_DOWN:
            break;

        case DR16RemoteControl::SwitchStatus3Pos::SWITCH_MIDDLE:
            break;

        case DR16RemoteControl::SwitchStatus3Pos::SWITCH_UP:
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
            fp32 fdbData[2] = {
                GSRLMath::normalizeDeltaAngle(m_pitchTargetAngle - m_pitchMotor->getCurrentAngle()),
                -m_pitchMotor->getCurrentAngularVelocity()};
            vofa.writeData(m_pitchMotor->getCurrentAngle());
            vofa.writeData(m_pitchTargetAngle);

            m_pitchMotor->externalClosedloopControl(0.0f, fdbData, 2);
        }

        default:
            break;
    }
}

void Gimbal::yawControl()
{
    switch (m_gimbalMode) {
        case GIMBAL_NO_FORCE:
            m_yawMotor->openloopControl(0.0f);
            break;

        case CALIBRATION:
            // m_yawMotor->setMotorZeroPosition(); //设定零点
            break;

        case MANUAL_CONTROL:
        case AUTO_CONTROL: { // 手动控制和自动控制都使用同样的闭环控制
            fp32 fdbData[2] = {
                GSRLMath::normalizeDeltaAngle(m_yawTargetAngle - m_yawMotor->getCurrentAngle()),
                -m_yawMotor->getCurrentAngularVelocity()};
            m_yawMotor->externalClosedloopControl(0.0f, fdbData, 2);

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

uint8_t Gimbal::transmitGimbalMotorDataViaUsb()
{
    // header 0
    uint32_t txbufIndex = 0;
    // After the assignment, `txbuf_index` becomes `1`.
    m_usbTxBuf[txbufIndex++] = m_usbTxSOF; // SOF 0x3A, remember to check twice

    // Roll angle (1-4)
    const float rollAngle = m_imu->getEulerAngle().x;
    memcpy(m_usbTxBuf + txbufIndex, &rollAngle, sizeof(float));
    txbufIndex += sizeof(float);

    // Pitch motor encoder angle (5 - 8)
    const float pitchAngle = m_pitchMotor->getCurrentAngle();
    memcpy(m_usbTxBuf + txbufIndex, &pitchAngle, sizeof(float)); // rad unit
    txbufIndex += sizeof(float);

    // Yaw motor encoder angle (9 - 12)
    const float yawAngle = m_yawMotor->getCurrentAngle();
    memcpy(m_usbTxBuf + txbufIndex, &yawAngle, sizeof(float)); // rad unit
    txbufIndex += sizeof(float);

    // the quaternion of IMU (13 - 28)
    const float *quaternion = m_imu->getQuaternion();
    memcpy(m_usbTxBuf + txbufIndex, quaternion, sizeof(float) * 4);
    txbufIndex += sizeof(float) * 4;

    // end of frame (29)
    m_usbTxBuf[txbufIndex] = m_usbTxEOF; // EOF 0xAA
    txbufIndex += 1;

    return CDC_Transmit_FS(m_usbTxBuf, static_cast<uint16_t>(txbufIndex));
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
