// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
#include <arm/Arm.h>

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
      m_encoder(ArmConstants::kAbsEncoderId),
      m_feedforward(ArmConstants::kFFks,
                    ArmConstants::kFFkg,
                    ArmConstants::kFFkV,
                    ArmConstants::kFFkA),
      
    Linear{1}
    // arm_pigeon{9, "NKCANivore"}
{
    auto armAngleConfig = ctre::phoenix6::configs::TalonFXConfiguration();

    armAngleConfig.CurrentLimits.SupplyCurrentLimitEnable = ArmConstants::kArmEnableCurrentLimit;
    armAngleConfig.CurrentLimits.SupplyCurrentLimit = ArmConstants::kArmContinuousCurrentLimit;
    armAngleConfig.CurrentLimits.SupplyCurrentThreshold = ArmConstants::kArmPeakCurrentLimit;
    armAngleConfig.CurrentLimits.SupplyTimeThreshold = ArmConstants::kArmPeakCurrentDuration;

    m_motor.GetConfigurator().Apply(armAngleConfig);
    m_motor.SetNeutralMode(ctre::phoenix6::signals::NeutralModeValue::Brake);

    m_encoder.SetPositionOffset(ArmConstants::kArmAngleOffset/360.0);
    m_encoder.SetDistancePerRotation(360);
    // m_encoder.SetPositionOffset(0.2);
    Linear.SetBounds(units::time::microsecond_t{ArmConstants::kLinearMax}, 
                    0_ms, 
                    0_ms,  
                    0_ms,  
                    units::time::microsecond_t{ArmConstants::kLinearMin});
  
    //Make pigeon kind of absolut
    arm_pigeon.SetYaw(units::angle::degree_t{m_encoder.GetDistance()});
    
    GetController().SetTolerance(ArmConstants::kControllerTolerance);
    // Start m_arm in neutral position
    SetGoal(State{units::degree_t(80.0), 0_rad_per_s});
    // arm_pigeon.Reset();
    //time_brake_released = frc::GetTime();
    // frc::SmartDashboard::PutNumber("Angle",100);

    //Linear.SetSpeed()
}

void ArmSubsystem::UseOutput(double output, State setpoint) {
  // Calculate the feedforward from the sepoint
  units::volt_t feedforward =
      m_feedforward.Calculate(setpoint.position, setpoint.velocity);
  frc::GetTime();

  // Output will be 0 if disabled.
  if(fabs(output) < 1e-6 || GetController().AtGoal())
  {
    m_motor.SetVoltage(units::volt_t{0.0});
      }
  else
  { 
    // Add the feedforward to the PID output to get the motor output
    m_motor.SetVoltage(units::volt_t{output} + feedforward);
  }
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

// void ArmSubsystem::get_pigeon(){
//     arm_pigeon.GetAccumGyroY();
// }

void ArmSubsystem::printLog()
{
    frc::SmartDashboard::PutNumber("ARM_ENC_ABS", GetMeasurement().value());  
    /*
    frc::SmartDashboard::PutNumber("ARM_Motor_PID", ( GetController().Calculate(units::degree_t{m_encoder.GetAbsolutePosition()})));
    frc::SmartDashboard::PutNumber("ARM_setpoint", GetController().GetSetpoint().position.value());*/
    // frc::SmartDashboard::PutNumber("ARM_Pigeon_Gyro_Y", arm_pigeon.GetAccumGyroY().GetValueAsDouble());
    // frc::SmartDashboard::PutNumber("ARM_Pigeon_Gyro_Y", arm_pigeon.GetAngle());
    // frc::SmartDashboard::PutNumber("ARM_Pigeon_Gravity_Z",arm_pigeon.GetGravityVectorZ().GetValueAsDouble());
    // frc::SmartDashboard::PutNumber("ARM_Pigeon_Gravity_Y",arm_pigeon.GetGravityVectorY().GetValueAsDouble());
    // frc::SmartDashboard::PutNumber("ARM_Pigeon_Gravity_X",arm_pigeon.GetGravityVectorX().GetValueAsDouble());
    /*
    frc::SmartDashboard::PutNumber("Brake Time", (frc::GetTime() - time_brake_released).value());
    frc::SmartDashboard::PutNumber("Arm State", m_ArmState);*/
    
    // Linear.SetBounds(units::time::microsecond_t{ArmConstants::kLinearMax}, 
    //                 0_ms, 
    //                 0_ms, 
    //                 0_ms, 
    //                 units::time::microsecond_t{ArmConstants::kLinearMin});

}

units::degree_t ArmSubsystem::GetMeasurement() {
  return units::degree_t{(m_encoder.GetDistance())};
}

void  ArmSubsystem::handle_Setpoint(units::angle::degree_t setpoint){
  if(fabs((setpoint - GetController().GetGoal().position).value()) >= 1e-1)
  {

    m_ArmState = ArmConstants::BRAKED;
    SetGoal(setpoint);
    arm_Brake_Out();
  }

  switch (m_ArmState)
  {
  case ArmConstants::BRAKED:
  {
    if ((frc::GetTime() - time_brake_released).value() > 0.5)
    {
      m_ArmState = ArmConstants::MOVING;
      Enable();
    }
  break;
  }
  case ArmConstants::MOVING:
  {
    if((GetController().GetGoal().position - GetMeasurement()) <
           units::angle::degree_t(3))
    {
      arm_Brake_In();
      m_ArmState = ArmConstants::MOVING;
    }
    if(GetController().AtGoal())
    {
      m_ArmState = ArmConstants::DONE;
    }
  break;
  }
  case ArmConstants::DONE:
  {
    Disable();
    break;
  }
  default:
    break;
}
}



