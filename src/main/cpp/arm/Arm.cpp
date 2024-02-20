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
    Linear{3}
    // arm_pigeon{9, "NKCANivore"}
{
    
    auto armAngleConfig = ctre::phoenix6::configs::TalonFXConfiguration();

    armAngleConfig.CurrentLimits.SupplyCurrentLimitEnable = ArmConstants::kArmEnableCurrentLimit;
    armAngleConfig.CurrentLimits.SupplyCurrentLimit = ArmConstants::kArmContinuousCurrentLimit;
    armAngleConfig.CurrentLimits.SupplyCurrentThreshold = ArmConstants::kArmPeakCurrentLimit;
    armAngleConfig.CurrentLimits.SupplyTimeThreshold = ArmConstants::kArmPeakCurrentDuration;


    m_motor.GetConfigurator().Apply(armAngleConfig);
    m_motor.SetNeutralMode(ctre::phoenix6::signals::NeutralModeValue::Brake);

    m_encoder.SetDistancePerRotation(360);
    // m_encoder.SetPositionOffset(0.2);
    Linear.SetBounds(units::time::microsecond_t{2.0}, units::time::microsecond_t{1.8}, units::time::microsecond_t {1.5},  units::time::microsecond_t{1.2},  units::time::microsecond_t{1.0});

    GetController().SetTolerance(ArmConstants::kControllerTolerance);
    // Start arm in neutral position
    SetGoal(State{units::degree_t(20.0), 0_rad_per_s});
    // arm_pigeon.Reset();
}

void ArmSubsystem::UseOutput(double output, State setpoint) {
  // Calculate the feedforward from the sepoint
  units::volt_t feedforward =
      m_feedforward.Calculate(setpoint.position, setpoint.velocity);

  // Output will be 0 if disabled.
  frc::SmartDashboard::PutNumber("Actual output", output);
  frc::SmartDashboard::PutNumber("Actual feed", feedforward.value());
  if(fabs(output) < 1e-6)
  {
    m_motor.SetVoltage(units::volt_t{0.0});
    // arm_Brake_In();
  }
  else
  { 
    //  if(arm_Brake_Out() == false){
    // Add the feedforward to the PID output to get the motor output
    m_motor.SetVoltage(units::volt_t{output} + feedforward);
    // }
  }
}

void  ArmSubsystem::arm_Brake_In()
{
   Linear.Set(0);
   
}

bool  ArmSubsystem::arm_Brake_Out()
{
  Linear.Set(0.5);

  return true;
}

// void ArmSubsystem::get_pigeon(){
//     arm_pigeon.GetAccumGyroY();
// }

// void ArmSubsystem::SetGoal(Distance_t setpoint) {

// }

void ArmSubsystem::printLog()
{
    frc::SmartDashboard::PutNumber("ARM_ENC_ABS", GetMeasurement().value());   
    frc::SmartDashboard::PutNumber("ARM_PID_Error", (GetController().GetPositionError().value()));
    // frc::SmartDashboard::PutNumber("ARM_ff", (m_feedforward.Calculate()));
    frc::SmartDashboard::PutNumber("ARM_Motor_PID", ( GetController().Calculate(units::degree_t{m_encoder.GetAbsolutePosition()})));
    // frc::SmartDashboard::PutNumber("ARM_Motor_Voltage", (m_motor.GetMotorVoltage().GetValueAsDouble()));
    frc::SmartDashboard::PutNumber("ARM_setpoint", GetController().GetSetpoint().position.value());
    // frc::SmartDashboard::PutNumber("ARM_Pigeon_Gyro_Y", arm_pigeon.GetAccumGyroY().GetValueAsDouble());
    // frc::SmartDashboard::PutNumber("ARM_Pigeon_Gyro_Y", arm_pigeon.GetAngle());



}

units::degree_t ArmSubsystem::GetMeasurement() {
  // return units::degree_t{360 * (m_encoder.GetAbsolutePosition())};
  return units::degree_t{(m_encoder.GetDistance())};
}