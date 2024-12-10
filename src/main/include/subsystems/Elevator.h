#pragma once

#include <Eigen/Core>
#include "frc/DataLogManager.h"
#include "wpi/DataLog.h"
#include <ctre/phoenix6/Pigeon2.hpp>
#include <ctre/phoenix6/TalonFX.hpp>
#include <frc/DutyCycleEncoder.h>
#include <frc/Encoder.h>
#include <frc/controller/ElevatorFeedforward.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc2/command/ProfiledPIDSubsystem.h>
#include <units/angle.h>
#include <units/time.h>
#include <units/voltage.h>
#include <frc/simulation/SimDeviceSim.h>
#include <frc/simulation/BatterySim.h>
#include <frc/simulation/ElevatorSim.h>
#include <frc/simulation/EncoderSim.h>

#include "Constants.hpp"
#include <frc/DigitalInput.h>
#include <frc/Servo.h>

namespace ElevatorConstants
{

enum ElevatorState
{
    START_ELEVATOR,
    BRAKED,
    // UNBRAKED,
    MOVING,
    DONE
};

const double kAngleP = 0.2;
const double kAngleI = 0.045;
const double kAngleD = 0.0015; // 0.0001
const double kIZone = 1.0;
const auto kElevatorVelLimit = units::degrees_per_second_t(140.0);
const auto kElevatorAccelLimit = units::angular_acceleration::degrees_per_second_squared_t(120.0);
const auto kControllerTolerance = units::degree_t(1.0);
const int kAngleMotorId = 5;
const int kAbsEncoderIdL = 1;
const int kAbsEncoderIdR = 0;

const int kAngleEncoderPulsePerRev = 2048;
const auto kFFks = units::volt_t(0.23);                               // Volts static (motor)
const auto kFFkg = units::volt_t(0.1);                                // Volts
const auto kFFkV = units::unit_t<frc::ElevatorFeedforward::kv_unit>(2.3);  // volts*s/rad
const auto kFFkA = units::unit_t<frc::ElevatorFeedforward::ka_unit>(0.01); // volts*s^2/rad

const bool kElevatorEnableCurrentLimit = true;
const int kElevatorContinuousCurrentLimit = 35;
const int kElevatorPeakCurrentLimit = 60;
const double kElevatorPeakCurrentDuration = 0.1;

const int kLinearMax = 1100; // 1140
const int kLinearMin = 1060; // 1100
const double kElevatorAngleOffsetL = 282.0;
const double kElevatorAngleOffsetR = 226.0;

const double kElevatorAngleStarting = 80.0;  // With offset
const double kElevatorAngleDriving = 30.0;   // With offset
const double kElevatorAngleIntake = -3;      // with offset
const double kElevatorAngleShootClose = 5.0; // with offset 0.0
const double kElevatorAngleShootFar = 31;    // with offset

//new
const int kElevatorGearing;
const int kCarriageMass;
const int kElevatorDrumRadius;
const int kMinElevatorHeight;
const int kMaxElevatorHeight;
const int kElevatorKp;
const int kElevatorKi;
const int kElevatorKd;
const int kElevatorkS;
const int kElevatorkG;
const int kElevatorkV;
const int kElevatorkA;
const int kEncoderAChannel;
const int kEncoderBChannel;
const int kMotorPort;




} // namespace ElevatorConstants

class ElevatorSubsystem : public frc2::ProfiledPIDSubsystem<units::degrees>
{
    using State = frc::TrapezoidProfile<units::degrees>::State;

  public:
    ElevatorSubsystem();
    void printLog();
    void elevator_Brake_In();
    void elevator_Brake_Out();
    void handle_Setpoint(units::angle::degree_t);
    void Emergency_Stop();
    // void get_pigeon();s
    void UseOutput(double output, State setpoint) override;
    units::degree_t GetMeasurement() override;
    bool isOverLimit();

    void SimulationPeriodic() override;

    units::time::second_t time_brake_released;
    ElevatorConstants::ElevatorState m_ElevatorState;

  private:
    int m_id;

    ctre::phoenix6::hardware::TalonFX m_motor;
    frc::ElevatorFeedforward m_feedforward;
    frc::DutyCycleEncoder m_encoderR;
    frc::DutyCycleEncoder m_encoderL;
    wpi::log::DoubleLogEntry m_AngleLog;
    wpi::log::DoubleLogEntry m_SetPointLog;
    wpi::log::IntegerLogEntry m_StateLog;
    wpi::log::DoubleLogEntry m_MotorCurrentLog;
    wpi::log::DoubleLogEntry m_MotorVoltageLog;
    ctre::phoenix6::hardware::Pigeon2 elevator_pigeon{9, "NKCANivore"};
    float ELEVATOR_Angle;
    frc::PWM Linear;
    frc::DigitalInput Kill{4};

    // simulation fields
    frc::Timer m_simTimer;

    Eigen::Matrix<double, 2, 2> m_A;
    Eigen::Matrix<double, 2, 1> m_B;
    Eigen::Vector<double, 2> m_c;
    Eigen::Vector<double, 2> m_d;
    Eigen::Vector<double, 2> m_x;

    //Sim additions 
    // This gearbox represents a gearbox containing 4 Vex 775pro motors.
  frc::DCMotor m_elevatorGearbox = frc::DCMotor::Vex775Pro(4);

  // Standard classes for controlling our elevator
  frc::TrapezoidProfile<units::meters>::Constraints m_constraints{2.45_mps,
                                                                  2.45_mps_sq};
  frc::ProfiledPIDController<units::meters> m_controller{
      ElevatorConstants::kElevatorKp, ElevatorConstants::kElevatorKi, ElevatorConstants::kElevatorKd,
      m_constraints};

  frc::ElevatorFeedforward m_feedforward{
      ElevatorConstants::kElevatorkS, ElevatorConstants::kElevatorkG, ElevatorConstants::kElevatorkV,
      ElevatorConstants::kElevatorkA};
  frc::Encoder m_encoder{ElevatorConstants::kEncoderAChannel,
                         ElevatorConstants::kEncoderBChannel};
  frc::PWMSparkMax m_motor{ElevatorConstants::kMotorPort};
  frc::sim::PWMSim m_motorSim{m_motor};

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

  // Create a Mechanism2d display of an elevator
  frc::Mechanism2d m_mech2d{20, 50};
  frc::MechanismRoot2d* m_elevatorRoot =
      m_mech2d.GetRoot("Elevator Root", 10, 0);
  frc::MechanismLigament2d* m_elevatorMech2d =
      m_elevatorRoot->Append<frc::MechanismLigament2d>(
          "Elevator", m_elevatorSim.GetPosition().value(), 90_deg);
};