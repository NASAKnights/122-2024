#pragma once

#include <frc/Encoder.h>
#include <frc/controller/ArmFeedforward.h>
#include <ctre/phoenix6/TalonFX.hpp>
#include <frc2/command/ProfiledPIDSubsystem.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <units/angle.h>
#include <frc/DutyCycleEncoder.h>
#include <ctre/phoenix6/Pigeon2.hpp>


#include "Constants.hpp"
#include <frc/Servo.h>


namespace ArmConstants {
const double kAngleP = 0.75;
const double kAngleI = 0.0;
const double kAngleD = 0.005;
const auto kArmVelLimit = units::degrees_per_second_t(50.0);
const auto kArmAccelLimit = units::unit_t<units::compound_unit<units::angular_velocity::degrees_per_second, units::inverse<units::time::seconds>>>(50.0);
const auto kControllerTolerance = units::degree_t(0.5);
const int kAngleMotorId = 1;
const int kAbsEncoderId = 1;
const int kAngleEncoderPulsePerRev = 2048;
const auto kFFks = units::volt_t(0.18); // Volts static (motor)
const auto kFFkg = units::volt_t(1.15); // Volts
const auto kFFkV = units::unit_t<frc::ArmFeedforward::kv_unit>(2.26); // volts*s/rad
const auto kFFkA = units::unit_t<frc::ArmFeedforward::ka_unit>(0.06); // volts*s^2/rad


const bool kArmEnableCurrentLimit = true;
const int kArmContinuousCurrentLimit = 35;
const int kArmPeakCurrentLimit = 60;
const double kArmPeakCurrentDuration = 0.1;

} // namespace ArmConstants


/**
 * A robot arm subsystem that moves with a motion profile.
 */
class ArmSubsystem : public frc2::ProfiledPIDSubsystem<units::degrees> {
  using State = frc::TrapezoidProfile<units::degrees>::State;

 public:
  ArmSubsystem();
  void printLog();
  void arm_Brake_In();
  bool arm_Brake_Out();
  // void get_pigeon();

  void UseOutput(double output, State setpoint) override;
  units::degree_t GetMeasurement() override;

 private:
  ctre::phoenix6::hardware::TalonFX m_motor;
  frc::ArmFeedforward m_feedforward;
  frc::DutyCycleEncoder m_encoder;	
  frc::Servo Linear;
  // ctre::phoenix6::hardware::Pigeon2 arm_pigeon{9, "NKCANivore"};
;


};