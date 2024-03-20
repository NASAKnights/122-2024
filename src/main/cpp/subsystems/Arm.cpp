// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
#include "subsystems/Arm.h"

using State = frc::TrapezoidProfile<units::degrees>::State;
using degrees_per_second_squared_t = units::unit_t<units::compound_unit<units::angular_velocity::degrees_per_second, units::inverse<units::time::seconds>>>;

ArmSubsystem::ArmSubsystem()
    : frc2::ProfiledPIDSubsystem<units::degrees>(
          frc::ProfiledPIDController<units::degrees>(
            ArmConstants::kAngleP,
            ArmConstants::kAngleI,
            ArmConstants::kAngleD,
            frc::TrapezoidProfile<units::degrees>::Constraints(
                ArmConstants::kArmVelLimit,
                ArmConstants::kArmAccelLimit), 5_ms)),
      m_motor(ArmConstants::kAngleMotorId),
      m_encoderL(ArmConstants::kAbsEncoderIdL),
      m_encoderR(ArmConstants::kAbsEncoderIdR),
      m_feedforward(ArmConstants::kFFks,
                    ArmConstants::kFFkg,
                    ArmConstants::kFFkV,
                    ArmConstants::kFFkA),
      
    Linear{1}
    // arm_pigeon{9, "NKCANivore"}
{
    auto armAngleConfig = ctre::phoenix6::configs::TalonFXConfiguration();
    GetController().SetIZone(ArmConstants::kIZone);

    armAngleConfig.CurrentLimits.SupplyCurrentLimitEnable = ArmConstants::kArmEnableCurrentLimit;
    armAngleConfig.CurrentLimits.SupplyCurrentLimit = ArmConstants::kArmContinuousCurrentLimit;
    armAngleConfig.CurrentLimits.SupplyCurrentThreshold = ArmConstants::kArmPeakCurrentLimit;
    armAngleConfig.CurrentLimits.SupplyTimeThreshold = ArmConstants::kArmPeakCurrentDuration;

    m_motor.GetConfigurator().Apply(armAngleConfig);
    m_motor.SetNeutralMode(ctre::phoenix6::signals::NeutralModeValue::Brake);
    // m_encoderL.Reset();
    // m_encoderR.Reset();

    m_encoderL.SetPositionOffset(ArmConstants::kArmAngleOffsetL/360.0);
    m_encoderL.SetDistancePerRotation(360);
    m_encoderR.SetPositionOffset(ArmConstants::kArmAngleOffsetR/360.0);
    m_encoderR.SetDistancePerRotation(-360);
   
    Linear.SetBounds(units::time::microsecond_t{ArmConstants::kLinearMax}, 
                    0_ms, 
                    0_ms,  
                    0_ms,  
                    units::time::microsecond_t{ArmConstants::kLinearMin});
  
    //Make pigeon kind of absolut
    arm_pigeon.SetYaw(units::angle::degree_t{m_encoderL.GetDistance()});
    
    GetController().SetTolerance(ArmConstants::kControllerTolerance);
    // Start m_arm in neutral position
    SetGoal(State{units::degree_t(80.0), 0_rad_per_s});
    // arm_pigeon.Reset();
    //time_brake_released = frc::GetTime();

    //Linear.SetSpeed()
}

void ArmSubsystem::UseOutput(double output, State setpoint) {
  // Calculate the feedforward from the sepoint
  units::volt_t feedforward =
      m_feedforward.Calculate(setpoint.position, setpoint.velocity);

  // Output will be 0 if disabled.
  // if(fabs(output) < 1e-6 || GetController().AtGoal())
  // {
    // m_motor.SetVoltage(units::volt_t{0.0});
  // }
  // else
  // { 
    // Add the feedforward to the PID output to get the motor output
    m_motor.SetVoltage(units::volt_t{output} + feedforward);
  // }
}

void ArmSubsystem::arm_Brake_In(){
  Linear.SetSpeed(1);
  m_ArmState = ArmConstants::BRAKED;
}
void ArmSubsystem::arm_Brake_Out()
{ 
  time_brake_released = frc::GetTime();
  Linear.SetSpeed(-1);
}

void ArmSubsystem::Emergency_Stop(){

  if(Kill.Get()== true){
    m_motor.StopMotor();
  }
}

bool ArmSubsystem::isOverLimit() {
  return Kill.Get();
}

void ArmSubsystem::printLog()
{
    frc::SmartDashboard::PutNumber("ARM_ENC_ABS", GetMeasurement().value()); 
    // frc::SmartDashboard::PutNumber("ARM_ENC_ABS_r", (m_encoderR.GetAbsolutePosition()-m_encoderR.GetPositionOffset())*-360.0); 
    frc::SmartDashboard::PutNumber("armGoal_POS", GetController().GetGoal().position.value());
    // frc::SmartDashboard::PutNumber("armGoal_vel", GetController().GetGoal().velocity.value());
    // frc::SmartDashboard::PutNumber("ARM_Motor_PID", ( GetController().Calculate(units::degree_t{m_encoder.GetAbsolutePosition()})));
    frc::SmartDashboard::PutNumber("ARM_setpoint", GetController().GetSetpoint().position.value());
    /*
    frc::SmartDashboard::PutNumber("Brake Time", (frc::GetTime() - time_brake_released).value());
    frc::SmartDashboard::PutNumber("Arm State", m_ArmState);*/
    


}

units::degree_t ArmSubsystem::GetMeasurement() { // original get measurement function
  return units::degree_t{m_encoderR.GetDistance()};
}


// units::degree_t ArmSubsystem::GetMeasurement() { //with redunant encoders
//   units::degree_t avrage_encoder = (units::degree_t{(m_encoderL.GetDistance())}+units::degree_t{(m_encoderR.GetDistance())})/2.0;
//   if(m_encoderL.GetDistance() == -ArmConstants::kArmAngleOffsetL){
//    return units::degree_t{(m_encoderR.GetDistance())};
//   }
//  if(m_encoderR.GetDistance() == -ArmConstants::kArmAngleOffsetR){
//    return units::degree_t{(m_encoderL.GetDistance())};
//   }
//   return avrage_encoder;
// }

void  ArmSubsystem::handle_Setpoint(units::angle::degree_t setpoint){
  if(fabs((setpoint - GetController().GetGoal().position).value()) >= 1e-1 && m_ArmState != ArmConstants::BRAKED)
  {

    m_ArmState = ArmConstants::START_ARM;
    // SetGoal(setpoint);
    // Disable();
  }

  switch (m_ArmState)
  {
  case ArmConstants::START_ARM:
  {
  //  arm_Brake_Out();
    m_ArmState = ArmConstants::BRAKED;
    break;
  }
  case ArmConstants::BRAKED:
  {
  //  if ((frc::GetTime() - time_brake_released).value() > 0.2)
    // {
    m_ArmState = ArmConstants::MOVING;
    SetGoal(setpoint);
    Enable();
    // }
    break;
  }
  case ArmConstants::MOVING:
  {
    //if(fabs((GetController().GetGoal().position - GetMeasurement()).value()) <3)
    // {
      //arm_Brake_In();
      // m_ArmState = ArmConstants::MOVING;
    // }
    if(GetController().AtGoal())
    {
      m_ArmState = ArmConstants::DONE;
    }
    break;
  }
  case ArmConstants::DONE:
  {
   // Disable();
    break;
  }
  default:
    break;
}
}



