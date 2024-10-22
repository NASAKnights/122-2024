// Copyright (c) FRC Team 122. All Rights Reserved.

#pragma once

#include <optional>

#include <frc/Joystick.h>
#include <frc/TimedRobot.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/PowerDistribution.h>
#include "frc/DataLogManager.h"
#include "wpi/DataLog.h"

#include <frc2/command/CommandPtr.h>
#include <frc2/command/CommandScheduler.h>
#include <frc2/command/InstantCommand.h>
#include <frc2/command/RunCommand.h>
#include <frc2/command/button/JoystickButton.h>
#include <frc2/command/button/POVButton.h>

#include <pathplanner/lib/commands/PathPlannerAuto.h>
#include <pathplanner/lib/auto/AutoBuilder.h>
#include <pathplanner/lib/util/HolonomicPathFollowerConfig.h>
#include <pathplanner/lib/auto/NamedCommands.h>

#include "subsystems/Climber.h"
#include "subsystems/SwerveDrive.hpp"
#include "subsystems/LEDController.h"
#include "subsystems/Arm.h"

#include "commands/shooter/Shoot.h"
#include "commands/shooter/SmartShoot.h"
#include "commands/Retract.h"
#include "commands/Extend.h"
#include "commands/intake/IntakeNote.h"
#include "commands/TrajectoryFollower.hpp"
#include "commands/ArmDown.h"
#include "commands/shooter/MotorRun.h"
#include "commands/intake/indexTheIntake.h"
#include "commands/Trapshoot.h"


#include "util/NKTrajectoryManager.hpp"

#include <units/angular_velocity.h>
#include <units/velocity.h>

#include <cmath>


class Robot : public frc::TimedRobot {
public:
  Robot();

  //
  // Robot Schedule methods
  //
  void RobotInit() override;
  void RobotPeriodic() override;
  void DisabledInit() override;
  void DisabledPeriodic() override;
  void AutonomousInit() override;
  void AutonomousPeriodic() override;
  void TeleopInit() override;
  void TeleopPeriodic() override;
  void TeleopExit() override;
  void TestPeriodic() override;
  void SimulationInit() override;
  void SimulationPeriodic() override;

private:
  // Have it empty by default so that if testing teleop it
  // doesn't have undefined behavior and potentially crash.
  std::optional<frc2::CommandPtr> m_autonomousCommand;
  void loadAutonomous();

  // Subsystems
  SwerveDrive m_swerveDrive;
  ArmSubsystem m_arm;
  

  Shooter m_shooter;
  Indexer m_indexer;
  Intake m_intake;
  LEDController m_LED_Controller;
  Climber m_climber;
  frc::PowerDistribution m_pdh = frc::PowerDistribution{1,frc::PowerDistribution::ModuleType::kRev};


  double autoName = 0;
  double ARM_Angel = 60.0;
  // double ARM_Speed = -120;

  double servo_angle = 100;
  float  distance;

  frc::DigitalInput autoColor{9};
  frc::DigitalInput autoSwitch8{8};
  frc::DigitalInput autoSwitch7{7};
  frc::DigitalInput autoSwitch6{6};

  // PS4 controllers
  frc::Joystick m_driverController{DriveConstants::kDriverPort};
  frc::Joystick m_operatorController{DriveConstants::kOperatorPort};

  // Power Distribution
  wpi::log::DoubleLogEntry m_VoltageLog;
  wpi::log::DoubleLogEntry m_CurrentLog;
  wpi::log::DoubleLogEntry m_PowerLog;
  wpi::log::DoubleLogEntry m_EnergyLog;
  wpi::log::DoubleLogEntry m_TemperatureLog;

  // Autocommands to load:
  frc::Pose2d autoBlueStart1;
  std::optional<frc2::CommandPtr> autoBlue1;
  // std::map<int, pathplanner::PathPlannerAuto> autoMap;
  std::map<int, std::pair<pathplanner::PathPlannerAuto, frc::Pose2d>> autoMap;

  //
  // Robot Container methods
  //
  void CreateRobot();
  void BindCommands();
  frc2::CommandPtr GetAutonomousCommand();
  void UpdateDashboard();
  void Arm_Position();
};
