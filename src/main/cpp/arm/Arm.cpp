// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "arm/Arm.h"
#include <frc/smartdashboard/SmartDashboard.h>

Arm::Arm() :
    m_AngleMotor{1},
    m_Encoder{1}
{
    armSlot0Configs.kP = ArmConstants::kArmP;
    armSlot0Configs.kI = ArmConstants::kArmI;
    armSlot0Configs.kD = ArmConstants::kArmD;

    armAngleConfig.Slot0 = armSlot0Configs;

    armCurrentLimitConfig.SupplyCurrentLimitEnable = ArmConstants::kArmEnableCurrentLimit;
    armCurrentLimitConfig.SupplyCurrentLimit = ArmConstants::kArmContinuousCurrentLimit;
    armCurrentLimitConfig.SupplyCurrentThreshold = ArmConstants::kArmPeakCurrentLimit;
    armCurrentLimitConfig.SupplyTimeThreshold = ArmConstants::kArmPeakCurrentDuration;

    armAngleConfig.CurrentLimits = armCurrentLimitConfig;

    m_AngleMotor.GetConfigurator().Apply(armAngleConfig);

    //TODO remove once we have proper encoder

    m_AngleMotor.SetPosition(units::turn_t{0.0});

}

// This method will be called once per scheduler run
void Arm::Periodic() {}


void Arm::armIn(){
//If no turn Motor turn motor on (we need to change direection)
//Check in loop  if either Limit switch or Encoder Reaches certain position
//If yes stop motor
     auto armRequest = ctre::phoenix6::controls::PositionVoltage{0_tr}.WithSlot(0);

    m_AngleMotor.SetControl(armRequest.WithPosition(units::turn_t{0.5}));
}

void Arm::armOut(){
//If no turn Motor turn motor on (we need to change direection)
//Check in loop  if either Limit switch or Encoder Reaches certain position
//If yes stop motor 
    auto armRequest = ctre::phoenix6::controls::PositionVoltage{0_tr}.WithSlot(0);

    m_AngleMotor.SetControl(armRequest.WithPosition(units::turn_t{0.0}));

}

void Arm::printLog(){
//frc::SmartDashboard::PutNumber("ArmEndoder ",m_Encoder);
    
}

void Arm::resetPivotEncoder() {
    m_AngleMotor.SetPosition(units::turn_t{0.0});
}

double Arm::getPivotAngle() {
    return double (m_AngleMotor.GetPosition().GetValue());
}