// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/SubsystemBase.h>
#include <ctre/phoenix6/TalonFX.hpp>
#include <frc/DigitalInput.h>
#include <rev/CANSparkMax.h>

class Shooter : public frc2::SubsystemBase {
 public:
  Shooter();
  void Periodic() override;

  void Shoot(double shootSpeed);
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

  rev::SparkMaxRelativeEncoder m_shootEncoderTop = m_shootMotorTop.GetEncoder();
  rev::SparkMaxRelativeEncoder m_shootEncoderBot = m_shootMotorBot.GetEncoder();

  bool running = false;
  int SHOOT_speed;
};
