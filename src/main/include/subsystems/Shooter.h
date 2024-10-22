// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include "frc/DataLogManager.h"
#include "wpi/DataLog.h"
#include <ctre/phoenix6/TalonFX.hpp>
#include <frc/DigitalInput.h>
#include <frc2/command/SubsystemBase.h>
#include <rev/CANSparkMax.h>

class Shooter : public frc2::SubsystemBase
{
  public:
    Shooter();
    void Periodic() override;

    void Shoot(double shootSpeed);
    void TrapShoot(double shootSpeed, double difference = 1);
    void stopShooter();

    double getSpeed();
    double getShuffleGoal();

    bool isRunning();
    bool atSetpoint();

  private:
    rev::CANSparkMax m_shootMotorTop;
    rev::CANSparkMax m_shootMotorBot;
    rev::SparkMaxPIDController m_shootPIDTop = m_shootMotorTop.GetPIDController();
    rev::SparkMaxPIDController m_shootPIDBot = m_shootMotorBot.GetPIDController();

    rev::SparkRelativeEncoder m_shootEncoderTop = m_shootMotorTop.GetEncoder();
    rev::SparkRelativeEncoder m_shootEncoderBot = m_shootMotorBot.GetEncoder();

    wpi::log::DoubleLogEntry m_VelocityTopLog;
    wpi::log::DoubleLogEntry m_VoltageTopLog;
    wpi::log::DoubleLogEntry m_CurrentTopLog;
    wpi::log::DoubleLogEntry m_VelocityBottomLog;
    wpi::log::DoubleLogEntry m_VoltageBottomLog;
    wpi::log::DoubleLogEntry m_CurrentBottomLog;

    bool running = false;
    int SHOOT_speed;
};
