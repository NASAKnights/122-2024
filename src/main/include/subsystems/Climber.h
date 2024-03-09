// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/SubsystemBase.h>
#include <ctre/phoenix6/TalonFX.hpp>
#include <frc/Servo.h>
#include <frc/DigitalInput.h>
#include <frc/smartdashboard/SmartDashboard.h>


enum ResetState {
  CLIMBER_EXTEND_START,
  CLIMBER_EXTEND_MOVING,
  CLIMBER_EXTEND_DONE,
  CLIMBER_EXTEND_BRAKE_DISENGAGE
};


class Climber : public frc2::SubsystemBase {
 public:
  Climber();

  /**
   * Will be called periodically whenever the CommandScheduler runs.
   */
  void Periodic() override;

  void engage();
  void disengage();
  void setAngle(double angle);

  void extend();
  void retract();
  
  void moveMotor();
  void stopMotor();
  void disableBrake();
  ResetState m_ClimberState;

 private:
  // Components (e.g. motor controllers and sensors) should generally be
  // declared private and exposed only through public methods.
  ctre::phoenix6::hardware::TalonFX climberMotor1;
  ctre::phoenix6::hardware::TalonFX climberMotor2;

  frc::Servo lockServo;
  frc::DigitalInput botLimit1{5};
  
  ctre::phoenix6::controls::Follower climberFollower;
  

  units::time::second_t time_brake_released;

};
