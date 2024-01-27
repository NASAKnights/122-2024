// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "arm/Arm.h"
#include <frc/smartdashboard/SmartDashboard.h>

Arm::Arm() :
m_AngleMotor{1},
m_Encoder{1}
{


}

// This method will be called once per scheduler run
void Arm::Periodic() {}


void armIn(){
//If no turn Motor turn motor on (we need to change direection)
//Check in loop  if either Limit switch or Encoder Reaches certain position
//If yes stop motor 


};

void armOut(){
//If no turn Motor turn motor on (we need to change direection)
//Check in loop  if either Limit switch or Encoder Reaches certain position
//If yes stop motor 
};

void printLog(){
//frc::SmartDashboard::PutNumber("ArmEndoder ",m_Encoder);
    
};