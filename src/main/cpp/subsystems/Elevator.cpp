// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
#include "subsystems/Elevator.h"

using State = frc::TrapezoidProfile<units::degrees>::State;
using degrees_per_second_squared_t = units::unit_t<
    units::compound_unit<units::angular_velocity::degrees_per_second, units::inverse<units::time::seconds>>>;

ElevatorSubsystem::ElevatorSubsystem()
    : frc2::ProfiledPIDSubsystem<units::degrees>(frc::ProfiledPIDController<units::degrees>(
          ElevatorConstants::kAngleP, ElevatorConstants::kAngleI, ElevatorConstants::kAngleD,
          frc::TrapezoidProfile<units::degrees>::Constraints(ElevatorConstants::kElevatorVelLimit, ElevatorConstants::kElevatorAccelLimit),
          5_ms)),
      m_motor(ElevatorConstants::kAngleMotorId), m_encoderL(ElevatorConstants::kAbsEncoderIdL),
      m_encoderR(ElevatorConstants::kAbsEncoderIdR),
      m_feedforward(ElevatorConstants::kFFks, ElevatorConstants::kFFkg, ElevatorConstants::kFFkV, ElevatorConstants::kFFkA),
      Linear{1} // elevator_pigeon{9, "NKCANivore"}
{
    auto elevatorAngleConfig = ctre::phoenix6::configs::TalonFXConfiguration();
    GetController().SetIZone(ElevatorConstants::kIZone);

    elevatorAngleConfig.CurrentLimits.SupplyCurrentLimitEnable = ElevatorConstants::kElevatorEnableCurrentLimit;
    elevatorAngleConfig.CurrentLimits.SupplyCurrentLimit = ElevatorConstants::kElevatorContinuousCurrentLimit;
    elevatorAngleConfig.CurrentLimits.SupplyCurrentThreshold = ElevatorConstants::kElevatorPeakCurrentLimit;
    elevatorAngleConfig.CurrentLimits.SupplyTimeThreshold = ElevatorConstants::kElevatorPeakCurrentDuration;

    m_motor.GetConfigurator().Apply(elevatorAngleConfig);
    m_motor.SetNeutralMode(ctre::phoenix6::signals::NeutralModeValue::Brake);

    m_encoderL.SetPositionOffset(ElevatorConstants::kElevatorAngleOffsetL / 360.0);
    m_encoderL.SetDistancePerRotation(360);
    m_encoderR.SetPositionOffset(ElevatorConstants::kElevatorAngleOffsetR / 360.0);
    m_encoderR.SetDistancePerRotation(-360);

    Linear.SetBounds(units::time::microsecond_t{ElevatorConstants::kLinearMax}, 0_ms, 0_ms, 0_ms,
                     units::time::microsecond_t{ElevatorConstants::kLinearMin});

    // Make pigeon kind of absolut
    elevator_pigeon.SetYaw(units::angle::degree_t{m_encoderL.GetDistance()});

    GetController().SetTolerance(ElevatorConstants::kControllerTolerance);
    // Start m_elevator in neutral position
    SetGoal(State{units::degree_t(80.0), 0_rad_per_s});

    wpi::log::DataLog &log = frc::DataLogManager::GetLog();
    m_AngleLog = wpi::log::DoubleLogEntry(log, "/Elevator/Angle");
    m_SetPointLog = wpi::log::DoubleLogEntry(log, "/Elevator/Setpoint");
    m_StateLog = wpi::log::IntegerLogEntry(log, "/Elevator/State");
    m_MotorCurrentLog = wpi::log::DoubleLogEntry(log, "/Elevator/MotorCurrent");
    m_MotorVoltageLog = wpi::log::DoubleLogEntry(log, "/Elevator/MotorVoltage");
}

void ElevatorSubsystem::UseOutput(double output, State setpoint)
{
    // Calculate the feedforward from the sepoint
    units::volt_t feedforward = m_feedforward.Calculate(setpoint.position, setpoint.velocity);
    m_motor.SetVoltage(units::volt_t{output} + feedforward);
}

void ElevatorSubsystem::elevator_Brake_In()
{
    Linear.SetSpeed(1);
    m_ElevatorState = ElevatorConstants::BRAKED;
}
void ElevatorSubsystem::elevator_Brake_Out()
{
    time_brake_released = frc::GetTime();
    Linear.SetSpeed(-1);
}

void ElevatorSubsystem::Emergency_Stop()
{
    if (Kill.Get() == true)
    {
        m_motor.StopMotor();
    }
}

bool ElevatorSubsystem::isOverLimit()
{
    return Kill.Get();
}

void ElevatorSubsystem::printLog()
{
    frc::SmartDashboard::PutNumber("ELEVATOR_ENC_ABS", GetMeasurement().value());
    frc::SmartDashboard::PutNumber("elevatorGoal_POS", GetController().GetGoal().position.value());
    frc::SmartDashboard::PutNumber("ELEVATOR_setpoint", GetController().GetSetpoint().position.value());
    m_AngleLog.Append(GetMeasurement().value());
    m_SetPointLog.Append(GetController().GetSetpoint().position.value());
    m_StateLog.Append(m_ElevatorState);
    m_MotorCurrentLog.Append(m_motor.GetSupplyCurrent().GetValue().value());
    m_MotorVoltageLog.Append(m_motor.GetMotorVoltage().GetValue().value());
}

units::degree_t ElevatorSubsystem::GetMeasurement()
{ // original get measurement function
    return units::degree_t{m_encoderR.GetDistance()};
}

void ElevatorSubsystem::SimulationPeriodic() {
  units::second_t dt = m_simTimer.Get();
  m_simTimer.Reset();
  
  //frc::SmartDashboard::PutNumber(std::to_string(m_id) + "Module Position",
  //                               m_driveSimPosition.Get());
}

void ElevatorSubsystem::handle_Setpoint(units::angle::degree_t setpoint)
{
    if (fabs((setpoint - GetController().GetGoal().position).value()) >= 1e-1 && m_ElevatorState != ElevatorConstants::BRAKED)
    {
        m_ElevatorState = ElevatorConstants::START_ELEVATOR;
        GetController().Reset(GetMeasurement());
    }

    switch (m_ElevatorState)
    {
    case ElevatorConstants::START_ELEVATOR: {
        m_ElevatorState = ElevatorConstants::BRAKED;
        break;
    }
    case ElevatorConstants::BRAKED: {
        m_ElevatorState = ElevatorConstants::MOVING;
        SetGoal(setpoint);
        Enable();
        break;
    }
    case ElevatorConstants::MOVING: {
        if (GetController().AtGoal())
        {
            m_ElevatorState = ElevatorConstants::DONE;
        }
        break;
    }
    case ElevatorConstants::DONE: {
        break;
    }
    default:
        break;
    }
}

  // Simulation classes help us simulate what's going on, including gravity.
  frc::sim::ElevatorSim m_elevatorSim{m_elevatorGearbox,
                                      ElevatorConstants::kElevatorGearing,
                                      ElevatorConstants::kCarriageMass,
                                      ElevatorConstants::kElevatorDrumRadius,
                                      ElevatorConstants::kMinElevatorHeight,
                                      ElevatorConstants::kMaxElevatorHeight,
                                      true,
                                      0_m,
                                      {0.01}};
  frc::sim::EncoderSim m_encoderSim{m_encoder};