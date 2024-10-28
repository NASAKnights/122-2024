// Copyright (c) FRC Team 122. All Rights Reserved.

#include "subsystems/SwerveModule.hpp"

#include <ctre/phoenix6/configs/Configs.hpp>
#include <ctre/phoenix6/controls/PositionVoltage.hpp>
#include <ctre/phoenix6/controls/VelocityVoltage.hpp>
#include <ctre/phoenix6/core/CoreCANcoder.hpp>
#include <ctre/phoenix6/core/CoreTalonFX.hpp>
#include <ctre/phoenix6/signals/SpnEnums.hpp>
#include <frc/RobotBase.h>
#include <frc/geometry/Rotation2d.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <units/angle.h>
#include <units/angular_acceleration.h>
#include <units/angular_velocity.h>

#include "Constants.hpp"

using namespace ctre::phoenix6;
using namespace ModuleConstants;

SwerveModule::SwerveModule(int driveMotorID, int steerMotorID,
                           int steerEncoderId, frc::Rotation2d angleOffset)
    : m_id{driveMotorID / 10}, m_driveMotor{driveMotorID, "NKCANivore"},
      m_steerMotor{steerMotorID, "NKCANivore"},
      m_steerEncoder{steerEncoderId, "NKCANivore"}, m_angleOffset{angleOffset},
      m_driveSim("TalonFX", driveMotorID), m_steerSim("TalonFX", steerMotorID),
      m_driveSimVelocity(m_driveSim.GetDouble("Velocity")),
      m_driveSimPosition(m_driveSim.GetDouble("Position")),
      m_steerSimPosition(m_steerSim.GetDouble("Position")) {
  configs::TalonFXConfiguration driveConfig{};
  configs::TalonFXConfiguration steerConfig{};
  configs::CANcoderConfiguration CANcoderConfig{};

  configs::ClosedLoopRampsConfigs driveClosedRamps{};
  driveClosedRamps.WithVoltageClosedLoopRampPeriod(0.4);
  driveClosedRamps.WithDutyCycleClosedLoopRampPeriod(0.4);
  driveClosedRamps.WithTorqueClosedLoopRampPeriod(0.4);
  driveConfig.ClosedLoopRamps = driveClosedRamps;

  CANcoderConfig.MagnetSensor.AbsoluteSensorRange =
      signals::AbsoluteSensorRangeValue::Signed_PlusMinusHalf;
  CANcoderConfig.MagnetSensor.SensorDirection =
      signals::SensorDirectionValue::CounterClockwise_Positive;
  CANcoderConfig.MagnetSensor.MagnetOffset =
      units::turn_t{m_angleOffset.Degrees()}.value();

  configs::Slot0Configs driveSlot0Configs{};
  configs::Slot0Configs steerSlot0Configs{};
  driveSlot0Configs.kP = kDriveP;
  driveSlot0Configs.kI = kDriveI;
  driveSlot0Configs.kD = kDriveD;
  driveSlot0Configs.kS = kDriveS;
  driveSlot0Configs.kV = kDriveV;
  driveSlot0Configs.kA = kDriveA;
  steerSlot0Configs.kP = kSteerP;
  steerSlot0Configs.kI = kSteerI;
  steerSlot0Configs.kD = kSteerD;
  driveConfig.Slot0 = driveSlot0Configs;
  steerConfig.Slot0 = steerSlot0Configs;

  configs::CurrentLimitsConfigs driveCurrentLimitConfig{};
  configs::CurrentLimitsConfigs steerCurrentLimitConfig{};
  driveCurrentLimitConfig.SupplyCurrentLimitEnable = kDriveEnableCurrentLimit;
  driveCurrentLimitConfig.SupplyCurrentLimit = kDriveContinuousCurrentLimit;
  driveCurrentLimitConfig.SupplyCurrentThreshold = kDrivePeakCurrentLimit;
  driveCurrentLimitConfig.SupplyTimeThreshold = kDrivePeakCurrentDuration;
  steerCurrentLimitConfig.SupplyCurrentLimitEnable = kSteerEnableCurrentLimit;
  steerCurrentLimitConfig.SupplyCurrentLimit = kSteerContinuousCurrentLimit;
  steerCurrentLimitConfig.SupplyCurrentThreshold = kSteerPeakCurrentLimit;
  steerCurrentLimitConfig.SupplyTimeThreshold = kSteerPeakCurrentDuration;
  driveConfig.CurrentLimits = driveCurrentLimitConfig;
  steerConfig.CurrentLimits = steerCurrentLimitConfig;

  // TODO: ensure this works, different invert method may be why we needed
  // negative PID values m_steerMotor.SetInverted(kSteerMotorInverted);
  // m_driveMotor.SetInverted(kDriveMotorInverted);
  driveConfig.MotorOutput.Inverted = kDriveMotorInverted;
  steerConfig.MotorOutput.Inverted = kSteerMotorInverted;

  

  m_steerMotor.SetNeutralMode(kSteerMotorNeutral);
  m_driveMotor.SetNeutralMode(kDriveMotorNeutral);

  steerConfig.Feedback.FeedbackRemoteSensorID = m_steerEncoder.GetDeviceID();
  steerConfig.Feedback.FeedbackSensorSource =
      signals::FeedbackSensorSourceValue::FusedCANcoder;
  steerConfig.Feedback.SensorToMechanismRatio = 1.0;
  steerConfig.Feedback.RotorToSensorRatio = kTurnGearRatio;
  steerConfig.ClosedLoopGeneral.ContinuousWrap = true;

  m_driveMotor.GetConfigurator().Apply(driveConfig);
  m_steerMotor.GetConfigurator().Apply(steerConfig);
  m_steerEncoder.GetConfigurator().Apply(CANcoderConfig);

  m_driveMotor.SetPosition(units::turn_t{0});
  ResetDriveEncoders();

  if constexpr (frc::RobotBase::IsSimulation()) {
    m_simTimer.Start();
  }
}

// This method will be called once per scheduler run
void SwerveModule::Periodic() {
  frc::SmartDashboard::PutNumber("Module " + std::to_string(m_id) + "/" +
                                     " Reported Angle",
                                 GetRotation().Degrees().value());
  frc::SmartDashboard::PutNumber("Module " + std::to_string(m_id) + "/" +
                                     " CANCoder Angle",
                                 GetAbsoluteRotation().Degrees().value());
  frc::SmartDashboard::PutNumber(
      "Module " + std::to_string(m_id) + "/" + " Velocity",
      (m_driveMotor.GetVelocity()).GetValue().value());
  frc::SmartDashboard::PutNumber(
      "Module " + std::to_string(m_id) + "/" + " Rotations",
      (m_driveMotor.GetPosition()).GetValue().value());

}

void SwerveModule::SimulationPeriodic() {
  units::second_t dt = m_simTimer.Get();
  m_simTimer.Reset();
  m_driveSimPosition.Set(m_driveSimPosition.Get() +
                         m_driveSimVelocity.Get() * dt.value());
  frc::SmartDashboard::PutNumber(std::to_string(m_id) + "Module Position",
                                 m_driveSimPosition.Get());
}

frc::SwerveModuleState SwerveModule::GetCurrentState() {
  return {m_driveMotor.GetVelocity().GetValue() * kDriveConversion,
          GetRotation()};
}

frc::SwerveModulePosition SwerveModule::GetPosition() {
  return {m_driveMotor.GetPosition().GetValue() * kDriveConversion,
          GetRotation()};
}

void SwerveModule::SetDesiredState(frc::SwerveModuleState state) {
  frc::Rotation2d rotation = GetRotation();

  state = frc::SwerveModuleState::Optimize(state, rotation);

  auto steerRequest = controls::PositionVoltage{0_tr}.WithSlot(0);
  auto driveRequest = controls::VelocityVoltage{0_tps}.WithSlot(0);

  m_steerMotor.SetControl(steerRequest.WithPosition(state.angle.Radians()));
  m_driveMotor.SetControl(
      driveRequest.WithVelocity(state.speed / kDriveConversion));

  if constexpr (frc::RobotBase::IsSimulation()) {
    m_steerSimPosition.Set(state.angle.Radians().value());
  }
}

void SwerveModule::ResetDriveEncoders() {
  m_driveMotor.SetPosition(0_tr, 50_ms);
}

frc::Rotation2d SwerveModule::GetRotation() {
  return units::radian_t{m_steerMotor.GetPosition().GetValue()};
}

frc::Rotation2d SwerveModule::GetAbsoluteRotation() {
  return units::radian_t{m_steerEncoder.GetAbsolutePosition().GetValue()};
}
