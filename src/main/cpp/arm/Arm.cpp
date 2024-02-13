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
    pid_Angle{10,0.1,0.1},
    voltRequest{units::volt_t {0.0}}
{
    armSlot0Configs.kP = ArmConstants::kArmP;
    armSlot0Configs.kI = ArmConstants::kArmI;
    armSlot0Configs.kD = ArmConstants::kArmD;
    //voltRequest = ctre::phoenix6::controls::VoltageOut(units::volt_t {0.0});

    armAngleConfig.Slot0 = armSlot0Configs;

    armCurrentLimitConfig.SupplyCurrentLimitEnable = ArmConstants::kArmEnableCurrentLimit;
    armCurrentLimitConfig.SupplyCurrentLimit = ArmConstants::kArmContinuousCurrentLimit;
    armCurrentLimitConfig.SupplyCurrentThreshold = ArmConstants::kArmPeakCurrentLimit;
    armCurrentLimitConfig.SupplyTimeThreshold = ArmConstants::kArmPeakCurrentDuration;

    armAngleConfig.CurrentLimits = armCurrentLimitConfig;
    
    m_AngleMotor.GetConfigurator().Apply(armAngleConfig);
    m_AngleMotor.SetNeutralMode(ctre::phoenix6::signals::NeutralModeValue::Brake);
    //m_AngleMotor.SetInverted(true);
    //TODO remove once we have proper encoder

    m_AngleMotor.SetPosition(units::turn_t{Trough_Encoder.GetAbsolutePosition()});
    pid_Angle.SetSetpoint(0);

}

// This method will be called once per scheduler run
void Arm::Periodic() {
   
   printLog();
   m_AngleMotor.SetPosition(units::turn_t{Trough_Encoder.GetAbsolutePosition()}); 
   

}
/*
0.2654 real 
2.8 value we use 
0.56 UP
*/

void Arm::arm_UP(){
          float des__Angle = 0.36;
        set_Arm_Position(des__Angle);
}

void Arm::arm_DOWN(){
         
          float des__Angle = 0.28;
          set_Arm_Position(des__Angle);
}


void Arm:: set_Arm_Position(float des__Angle){
          pid_Angle.SetSetpoint(des__Angle);
          pid_Angle.Calculate(Trough_Encoder.GetAbsolutePosition(), des__Angle);
          double temp = Trough_Encoder.GetAbsolutePosition();
          if (fabs(pid_Angle.GetPositionError()) > 0.01)
          {
            frc::SmartDashboard::PutNumber("ARM_enc_ABS",temp);   
            frc::SmartDashboard::PutNumber("Error_ARM_PID",(pid_Angle.GetPositionError()));
            frc::SmartDashboard::PutNumber("Voltage_ARM_PID",(pid_Angle.Calculate(Trough_Encoder.GetAbsolutePosition())));
            frc::SmartDashboard::PutNumber("Voltage_ARM_MOtor",(m_AngleMotor.GetMotorVoltage().GetValueAsDouble()));
            //m_AngleMotor.SetVoltage(units::volt_t {pid_Angle.Calculate(Trough_Encoder.GetAbsolutePosition(), des__Angle)});
            m_AngleMotor.SetControl(voltRequest.WithOutput(units::volt_t {pid_Angle.Calculate(Trough_Encoder.GetAbsolutePosition())}));
          }
          else m_AngleMotor.StopMotor();
}
    
void Arm::printLog(){
  
//frc::SmartDashboard::PutNumber("ARM_enc_ABS",Trough_Encoder.GetAbsolutePosition());   
//frc::SmartDashboard::PutNumber("Error_ARM_PID",(pid_Angle.GetPositionError()));
//frc::SmartDashboard::PutNumber("Voltage_ARM_PID",(pid_Angle.Calculate(Trough_Encoder.GetAbsolutePosition())));
//frc::SmartDashboard::PutNumber("Voltage_ARM_MOtor",(m_AngleMotor.GetMotorVoltage().GetValueAsDouble()));


}

void Arm::Set_Current() {

    pid_Angle.SetSetpoint(Trough_Encoder.GetAbsolutePosition());
    m_AngleMotor.StopMotor();
}

void Arm::resetPivotEncoder() {
    m_AngleMotor.SetPosition(units::turn_t{0.0});
}

double Arm::getPivotAngle() {
    return double (m_AngleMotor.GetPosition().GetValue());
}