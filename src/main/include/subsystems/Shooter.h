// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/SubsystemBase.h>
#include <ctre/phoenix6/TalonFX.hpp>
#include <frc/DigitalInput.h>

class Shooter : public frc2::SubsystemBase {
 public:
  Shooter();

  void Periodic() override;

  void Shoot();
  void stopShooter();

  double getSpeed();

  bool isRunning();

 private:
  // Components (e.g. motor controllers and sensors) should generally be
  // declared private and exposed only through public methods.
  ctre::phoenix6::hardware::TalonFX m_shooterMotorMain{6};
  ctre::phoenix6::hardware::TalonFX m_shooterMotorFollow{7};
  ctre::phoenix6::controls::Follower m_follower;

  ctre::phoenix6::controls::VelocityVoltage velocityControl;

  bool running = false;
};
