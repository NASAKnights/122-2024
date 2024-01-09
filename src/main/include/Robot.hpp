// Copyright (c) FRC Team 122. All Rights Reserved.

#pragma once

#include <optional>

#include <frc/Joystick.h>
#include <frc/TimedRobot.h>
#include <frc2/command/CommandPtr.h>
#include <frc2/command/InstantCommand.h>
#include <frc2/command/RunCommand.h>

#include "Subsystems/SwerveDrive.hpp"

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
  void TestPeriodic() override;
  void SimulationInit() override;
  void SimulationPeriodic() override;

private:
  // Have it empty by default so that if testing teleop it
  // doesn't have undefined behavior and potentially crash.
  std::optional<frc2::CommandPtr> m_autonomousCommand;

  // Subsystems
  SwerveDrive m_swerveDrive;

  // PS4 controllers
  frc::Joystick m_driverController{DriveConstants::kDriverPort};
  frc::Joystick m_operatorController{DriveConstants::kOperatorPort};

  //
  // Robot Container methods
  //
  void CreateRobot();
  void BindCommands();
  frc2::CommandPtr GetAutonomousCommand();
  void UpdateDashboard();
};
