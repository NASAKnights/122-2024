// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
#include <arm/Arm.h>

using State = frc::TrapezoidProfile<units::degrees>::State;
using degrees_per_second_squared_t = units::unit_t<units::compound_unit<units::angular_velocity::degrees_per_second, units::inverse<units::time::seconds>>>;

ArmSubsystem::ArmSubsystem()
    : frc2::ProfiledPIDSubsystem<units::degrees>(
          frc::ProfiledPIDController<units::degrees>(ArmConstants::kAngleP,
            ArmConstants::kAngleI,
            ArmConstants::kAngleD,
            frc::TrapezoidProfile<units::degrees>::Constraints(
                ArmConstants::kArmVelLimit,
                ArmConstants::kArmAccelLimit))),
      m_motor(ArmConstants::kAngleMotorId),
      m_encoder(ArmConstants::kAbsEncoderId),
      m_feedforward(ArmConstants::kFFks,
                    ArmConstants::kFFkg,
                    ArmConstants::kFFkV,
                    ArmConstants::kFFkA)
{
    auto armAngleConfig = ctre::phoenix6::configs::TalonFXConfiguration();

    armAngleConfig.CurrentLimits.SupplyCurrentLimitEnable = ArmConstants::kArmEnableCurrentLimit;
    armAngleConfig.CurrentLimits.SupplyCurrentLimit = ArmConstants::kArmContinuousCurrentLimit;
    armAngleConfig.CurrentLimits.SupplyCurrentThreshold = ArmConstants::kArmPeakCurrentLimit;
    armAngleConfig.CurrentLimits.SupplyTimeThreshold = ArmConstants::kArmPeakCurrentDuration;


    m_motor.GetConfigurator().Apply(armAngleConfig);
    m_motor.SetNeutralMode(ctre::phoenix6::signals::NeutralModeValue::Brake);

    m_encoder.SetDistancePerRotation(360);

    GetController().SetTolerance(ArmConstants::kControllerTolerance);
    // Start arm in neutral position
    SetGoal(State{units::degree_t(20.0), 0_rad_per_s});
}

void ArmSubsystem::UseOutput(double output, State setpoint) {
  // Calculate the feedforward from the sepoint
  units::volt_t feedforward =
      m_feedforward.Calculate(setpoint.position, setpoint.velocity);

  // Output will be 0 if disabled.
  if(output < 1e-6)
  {
    m_motor.SetVoltage(units::volt_t{0.0});
    //TODO: set brake
  }
  else
  {
    //TODO: unset brake
    //TODO: maybe only move if brake is unset
    // Add the feedforward to the PID output to get the motor output
    m_motor.SetVoltage(units::volt_t{output} + feedforward);
  }
}

void ArmSubsystem::printLog()
{
    frc::SmartDashboard::PutNumber("ARM_enc_ABS", m_encoder.GetAbsolutePosition());   
    frc::SmartDashboard::PutNumber("Error_ARM_PID", (GetController().GetPositionError().value()));
    frc::SmartDashboard::PutNumber("Voltage_ARM_PID", ( GetController().Calculate(units::degree_t{m_encoder.GetAbsolutePosition()})));
    frc::SmartDashboard::PutNumber("Voltage_ARM_Motor", (m_motor.GetMotorVoltage().GetValueAsDouble()));
}

units::degree_t ArmSubsystem::GetMeasurement() {
  return units::degree_t{m_encoder.GetDistance()};

}