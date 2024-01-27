// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/SubsystemBase.h>

#include <ctre/phoenix6/CANcoder.hpp>
#include <ctre/phoenix6/TalonFX.hpp>
#include <frc/DigitalInput.h>


class Arm : public frc2::SubsystemBase {
 public:
 Arm();

  void Periodic() override; 
 
 
  void armIn();
  void armOut();
  void printLog();

 private:
  ctre::phoenix6::hardware::TalonFX m_AngleMotor;
  //TODO: Change later to absolute
   ctre::phoenix6::hardware::CANcoder m_Encoder;
 

   frc::DigitalInput rightLimitSwitch {0};
   frc::DigitalInput leftLimitSwitch {1};
  
  // assume gearing is 1:1


};
