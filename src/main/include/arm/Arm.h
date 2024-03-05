#pragma once

#include <frc/Encoder.h>
#include <frc/controller/ArmFeedforward.h>
#include <ctre/phoenix6/TalonFX.hpp>
#include <frc2/command/ProfiledPIDSubsystem.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <units/angle.h>
#include <frc/DutyCycleEncoder.h>
#include <ctre/phoenix6/Pigeon2.hpp>
#include <units/time.h>


#include "Constants.hpp"
#include <frc/Servo.h>


namespace ArmConstants {

enum ArmState{
    BRAKED,
    // UNBRAKED,
    MOVING,
    DONE
  };  

const double kAngleP = 0.3;
const double kAngleI = 0.0;
const double kAngleD = 0.0;//0.0001
const auto kArmVelLimit = units::degrees_per_second_t(90.0);
const auto kArmAccelLimit = units::unit_t<units::compound_unit<units::angular_velocity::degrees_per_second, units::inverse<units::time::seconds>>>(90.0);
const auto kControllerTolerance = units::degree_t(3.0);
const int kAngleMotorId = 5;
const int kAbsEncoderId = 1;
const int kAngleEncoderPulsePerRev = 2048;
const auto kFFks = units::volt_t(0.18); // Volts static (motor)
const auto kFFkg = units::volt_t(0.25); // Volts
const auto kFFkV = units::unit_t<frc::ArmFeedforward::kv_unit>(2.26); // volts*s/rad
const auto kFFkA = units::unit_t<frc::ArmFeedforward::ka_unit>(0.06); // volts*s^2/rad

const bool kArmEnableCurrentLimit = true;
const int kArmContinuousCurrentLimit = 35;
const int kArmPeakCurrentLimit = 60;
const double kArmPeakCurrentDuration = 0.1;

const int kLinearMax = 1140;
const int kLinearMin = 1100;

const double kArmAngleOffset = 40.0;
const double kArmAngleStarting = 60.0; // With offset 
const double kArmAngleDriving = 30.0; // With offset
const double kArmAngleIntake = -1.5; //with offset
const double kArmAngleShootClose = 40.0; //with offset

} // namespace ArmConstants


/**
 * A robot m_arm subsystem that moves with a motion profile.
 */
class ArmSubsystem : public frc2::ProfiledPIDSubsystem<units::degrees> {
  using State = frc::TrapezoidProfile<units::degrees>::State;

 public:
  ArmSubsystem();
  void printLog();
  void arm_Brake_In();
  void arm_Brake_Out();
  void handle_Setpoint(units::angle::degree_t);
  // void get_pigeon();
  units::time::second_t time_brake_released;
    ArmConstants::ArmState m_ArmState;


  void UseOutput(double output, State setpoint) override;
  units::degree_t GetMeasurement() override;
  private:
  ctre::phoenix6::hardware::TalonFX m_motor;
  frc::ArmFeedforward m_feedforward;
  frc::DutyCycleEncoder m_encoder;	
  ctre::phoenix6::hardware::Pigeon2 arm_pigeon{9, "NKCANivore"};
  float ARM_Angle;
  frc::PWM Linear;

};