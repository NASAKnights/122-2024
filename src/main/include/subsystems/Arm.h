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
#include <frc/DigitalInput.h>


namespace ArmConstants {

enum ArmState{
    START_ARM,
    BRAKED,
    // UNBRAKED,
    MOVING,
    DONE
  };  

const double kAngleP = 0.045;
const double kAngleI = 0.045;
const double kAngleD = 0.0;//0.0001
const double kIZone = 3.0;
const auto kArmVelLimit = units::degrees_per_second_t(140.0);
const auto kArmAccelLimit = units::angular_acceleration::degrees_per_second_squared_t(120.0);
const auto kControllerTolerance = units::degree_t(1.0);
const int kAngleMotorId = 5;
const int kAbsEncoderIdL = 1;
const int kAbsEncoderIdR = 0;

const int kAngleEncoderPulsePerRev = 2048;
const auto kFFks = units::volt_t(0.23); // Volts static (motor)
const auto kFFkg = units::volt_t(0.0); // Volts
const auto kFFkV = units::unit_t<frc::ArmFeedforward::kv_unit>(2.26); // volts*s/rad
const auto kFFkA = units::unit_t<frc::ArmFeedforward::ka_unit>(0.06); // volts*s^2/rad

const bool kArmEnableCurrentLimit = true;
const int kArmContinuousCurrentLimit = 35;
const int kArmPeakCurrentLimit = 60;
const double kArmPeakCurrentDuration = 0.1;

const int kLinearMax = 1100;//1140
const int kLinearMin = 1060;//1100
const double kArmAngleOffsetL = 282.0;
const double kArmAngleOffsetR = 226.0;


const double kArmAngleStarting = 80.0; // With offset
const double kArmAngleDriving = 30.0; // With offset
const double kArmAngleIntake = -4.5; //with offset
const double kArmAngleShootClose = 0.0; //with offset 0.0
const double kArmAngleShootFar = 23; //with offset

// 3.0 ft, 0.0
// 6.0 ft, 17.5
// 9.0 ft, 22
// 12 ft, ??
// 15 ft, 30
// 20 ft, 33

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
  void Emergency_Stop();
  // void get_pigeon();s
  void UseOutput(double output, State setpoint) override;
  units::degree_t GetMeasurement() override;
  bool isOverLimit();

  units::time::second_t time_brake_released;
  ArmConstants::ArmState m_ArmState;
private:
  ctre::phoenix6::hardware::TalonFX m_motor;
  frc::ArmFeedforward m_feedforward;
  frc::DutyCycleEncoder m_encoderR;	
  frc::DutyCycleEncoder m_encoderL;	

  ctre::phoenix6::hardware::Pigeon2 arm_pigeon{9, "NKCANivore"};
  float ARM_Angle;
  frc::PWM Linear;
  frc::DigitalInput Kill{4};

};