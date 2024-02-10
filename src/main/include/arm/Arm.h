// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/SubsystemBase.h>

#include <ctre/phoenix6/CANcoder.hpp>
#include <ctre/phoenix6/TalonFX.hpp>
#include <frc/DigitalInput.h>

#include "Constants.hpp"
#include <frc/DutyCycleEncoder.h>
#include <frc/controller/PIDController.h>



class Arm : public frc2::SubsystemBase {
 public:
 Arm();

  void Periodic() override; 
 
 
  void arm_UP();
  void arm_DOWN();
  void printLog();
  void set_Arm_Position(int);
  void Set_Current();
  void resetPivotEncoder();
  double getPivotAngle();
  void InitializePID_ARM();


 private:
  ctre::phoenix6::hardware::TalonFX m_AngleMotor;
  ctre::phoenix6::configs::TalonFXConfiguration armAngleConfig;
  ctre::phoenix6::configs::CANcoderConfiguration armEncoderConfig;

  ctre::phoenix6::configs::Slot0Configs armSlot0Configs;
  ctre::phoenix6::configs::CurrentLimitsConfigs armCurrentLimitConfig;
  //TODO: Change later to absolute
  ctre::phoenix6::hardware::CANcoder m_Encoder;


  
  frc::PIDController pid_Angle;
  frc::DigitalInput rightLimitSwitch {3};
  frc::DigitalInput leftLimitSwitch {1};
  frc::DutyCycleEncoder Trough_Encoder {2};	
  
  // assume gearing is 1:1


};
