// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "arm/Arm.h"
#include <frc/smartdashboard/SmartDashboard.h>
#include <ctre/phoenix6/core/CoreTalonFX.hpp>
#include <iostream>

Arm::Arm() :
    m_AngleMotor{1},
    m_Encoder{1},
    pid_Angle{0.8,0.3,0.1}
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
    m_AngleMotor.SetNeutralMode(ctre::phoenix6::signals::NeutralModeValue::Brake);
 
    //TODO remove once we have proper encoder

    m_AngleMotor.SetPosition(units::turn_t{Trough_Encoder.GetAbsolutePosition()-0.1});
    
}

// This method will be called once per scheduler run
void Arm::Periodic() {

   printLog();
  m_AngleMotor.SetPosition(units::turn_t{Trough_Encoder.GetAbsolutePosition()-0.1}); 

}


void Arm::armIn(){
          int des__Angle = 0.24;
          pid_Angle.Calculate(Trough_Encoder.GetAbsolutePosition()-0.1, des__Angle);
          while (fabs(pid_Angle.GetPositionError()) > 0.1)
          {
            m_AngleMotor.SetVoltage(units::volt_t {pid_Angle.Calculate(Trough_Encoder.GetAbsolutePosition()-0.1, des__Angle)});

          }
          m_AngleMotor.StopMotor();
          
}

void Arm::armOut(){
         
          int des__Angle = 0.54;
          pid_Angle.Calculate(Trough_Encoder.GetAbsolutePosition()-0.1, des__Angle);
          while (fabs(pid_Angle.GetPositionError()) > 0.1)
          {
            m_AngleMotor.SetVoltage(units::volt_t {pid_Angle.Calculate(Trough_Encoder.GetAbsolutePosition()-0.1, des__Angle)});

          }
          m_AngleMotor.StopMotor();
}


void Arm:: set_Motor_Position(int des__Angle){
          pid_Angle.Calculate(Trough_Encoder.GetAbsolutePosition()-0.1, des__Angle);
          while (fabs(pid_Angle.GetPositionError()) > 0.1)
          {
            m_AngleMotor.SetVoltage(units::volt_t {pid_Angle.Calculate(Trough_Encoder.GetAbsolutePosition()-0.1, des__Angle)});

          }
          m_AngleMotor.StopMotor();
}
    
void Arm::printLog(){
frc::SmartDashboard::PutNumber("ARm_encoder_GetAbs",Trough_Encoder.GetAbsolutePosition()*360);   
frc::SmartDashboard::PutNumber("Error",fabs(pid_Angle.GetPositionError()));
}


void Arm::resetPivotEncoder() {
    m_AngleMotor.SetPosition(units::turn_t{0.0});
}

double Arm::getPivotAngle() {
    return double (m_AngleMotor.GetPosition().GetValue());
}