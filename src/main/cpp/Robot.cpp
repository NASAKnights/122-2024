// Copyright (c) FRC Team 122. All Rights Reserved.

#include "Robot.hpp"

#include <frc/smartdashboard/SmartDashboard.h>
#include <frc2/command/CommandScheduler.h>
#include <frc2/command/RunCommand.h>
#include <frc2/command/button/JoystickButton.h>
#include <units/angular_velocity.h>
#include <units/velocity.h>

#include "commands/TrajectoryFollower.hpp"
#include "frc2/command/InstantCommand.h"
#include "util/NKTrajectoryManager.hpp"
#include <arm/Arm.h>

Robot::Robot() { this->CreateRobot(); }

void Robot::RobotInit() 
{

};

/**
 * This function is called every 20 ms, no matter the mode. Use
 * this for items like diagnostics that you want to run during disabled,
 * autonomous, teleoperated and test.
 *
 * <p> This runs after the mode specific periodic functions, but before
 * LiveWindow and SmartDashboard integrated updating.
 */
void Robot::RobotPeriodic() {
  frc2::CommandScheduler::GetInstance().Run();
  this->UpdateDashboard();
}

/**
 * This function is called once each time the robot enters Disabled mode. You
 * can use it to reset any subsystem information you want to clear when the
 * robot is disabled.
 */
void Robot::DisabledInit() {
  
}


/**
 * This autonomous runs the autonomous command selected by your {@link
 * RobotContainer} class.
 */
void Robot::AutonomousInit() {
  m_autonomousCommand = this->GetAutonomousCommand();

  if (m_autonomousCommand) {
    m_autonomousCommand->Schedule();
  }
}

void Robot::AutonomousPeriodic() {}

void Robot::TeleopInit() {
  // This makes sure that the autonomous stops running when
  // teleop starts running. If you want the autonomous to
  // continue until interrupted by another command, remove
  // this line or comment it out.
  if (m_autonomousCommand) {
    m_autonomousCommand->Cancel();
  }
}

/**
 * This function is called periodically during operator control.
 */
void Robot::TeleopPeriodic() {
  // m_arm.Periodic();

}

void Robot::TeleopExit() {
  m_arm.arm_Brake_Out();
}

/**
 * This function is called periodically during test mode.
 */
void Robot::TestPeriodic() {}

/**
 * This function is called once when the robot is first started up.
 */
void Robot::SimulationInit() {}

/**
 * This function is called periodically whilst in simulation.
 */
void Robot::SimulationPeriodic() {}

/**
 * Initializes the robot subsystems and binds commands
 */
void Robot::CreateRobot() {
  // Initialize all of your commands and subsystems here
  m_swerveDrive.SetDefaultCommand(frc2::RunCommand(
      [this] {
        auto leftXAxis =
            MathUtilNK::calculateAxis(m_driverController.GetRawAxis(1),
                                      DriveConstants::kDefaultAxisDeadband);
        auto leftYAxis =
            MathUtilNK::calculateAxis(m_driverController.GetRawAxis(0),
                                      DriveConstants::kDefaultAxisDeadband);
        auto rightXAxis =
            MathUtilNK::calculateAxis(m_driverController.GetRawAxis(2),
                                      DriveConstants::kDefaultAxisDeadband);
        // frc::SmartDashboard::PutNumber("Joystick/Left X Axis", leftXAxis);
        // frc::SmartDashboard::PutNumber("Joystick/Left Y Axis", leftYAxis);
        // frc::SmartDashboard::PutNumber("Joystick/Right X Axis", rightXAxis);
        m_swerveDrive.Drive(frc::ChassisSpeeds::FromFieldRelativeSpeeds(
            -leftXAxis * DriveConstants::kMaxTranslationalVelocity,
            -leftYAxis * DriveConstants::kMaxTranslationalVelocity,
            -rightXAxis * DriveConstants::kMaxRotationalVelocity,
            m_swerveDrive.GetHeading()));
      },
      {&m_swerveDrive}));

    m_arm.SetDefaultCommand(frc2::RunCommand(
      [this] {
        m_arm.handle_Setpoint(units::degree_t(70));
      },
      {&m_arm}));

  // Configure the button bindings
  BindCommands();
  m_swerveDrive.ResetHeading();
  AddPeriodic([this] {
        m_arm.Periodic();
      }, 5_ms, 1_ms);

}

/**
 * Binds commands to Joystick buttons
 */
void Robot::BindCommands() {
  frc2::JoystickButton(&m_driverController, 1)
      .OnTrue(frc2::CommandPtr(frc2::InstantCommand([this] {
        return m_swerveDrive.ResetHeading();
      }))); // TODO assign as test

  frc2::JoystickButton(&m_operatorController, 6).OnFalse(frc2::CommandPtr(frc2::InstantCommand([this] {
        // m_arm.SetGoal(units::degree_t(m_arm.GetMeasurement()));
        m_arm.Disable();
      },
      {&m_arm}).ToPtr()))
      .WhileTrue(Shoot(&m_shooter, &m_indexer, &m_intake, &m_arm, -100, 78).ToPtr());

    frc2::JoystickButton(&m_operatorController, 3)
      .WhileTrue(intakeTake(&m_intake, &m_indexer, &m_arm).ToPtr());

  frc2::JoystickButton(&m_operatorController, 4)
    .OnTrue(frc2::RunCommand([this] {
      m_intake.runIntakeReverse();
    },{&m_intake}).ToPtr())
    .OnFalse(frc2::InstantCommand([this] {
      return m_intake.stopIntake();
    }, {&m_intake}).ToPtr());

  frc2::JoystickButton(&m_driverController, 2).OnTrue(frc2::InstantCommand(
      [this] {
        //108
        m_arm.handle_Setpoint(units::degree_t(120)); //108
     
      },
      {&m_arm}).ToPtr()
      ).OnFalse(frc2::CommandPtr(frc2::InstantCommand([this] {
        m_arm.Disable();
      },
      {&m_arm}).ToPtr()));

  frc2::JoystickButton(&m_driverController, 3).OnTrue(frc2::InstantCommand(
      [this] {
        //54
        //m_arm.SetGoal(units::degree_t(60)); //42.5
        m_arm.Enable();
      },
      {&m_arm}).ToPtr()
      ).OnFalse(frc2::CommandPtr(frc2::InstantCommand([this] {
        // m_arm.SetGoal(units::degree_t(m_arm.GetMeasurement()));
        m_arm.Disable();
      },
      {&m_arm}).ToPtr()));

}
/**
 * Returns the Autonomous Command
 */
frc2::CommandPtr Robot::GetAutonomousCommand() {
  return TrajectoryFollower(&m_swerveDrive,
                            &NKTrajectoryManager::GetTrajectory("NewPath"))
      .ToPtr();
  // return frc2::InstantCommand().ToPtr();
}

void Robot::DisabledPeriodic() {
  m_arm.Disable();
}

/**
 * Updates the data on the dashboard
 */
void Robot::UpdateDashboard() {
  // frc::SmartDashboard::PutNumber("driver X", m_driverController.GetX());
  // frc::SmartDashboard::PutNumber(
  //     "adjusted X",
  //     MathUtilNK::calculateAxis(m_driverController.GetX(),
  //                               DriveConstants::kDefaultAxisDeadband) *
  //         DriveConstants::kMaxTranslationalVelocity.value());
  // frc::SmartDashboard::PutNumber("Swerve Drive Heading",
  //                                m_swerveDrive.GetHeading().Degrees().value());
  frc::SmartDashboard::PutBoolean("Note?", m_indexer.hasNote());
  // m_swerveDrive.PrintNetworkTableValues();
  m_arm.printLog();
  
}
#ifndef RUNNING_FRC_TESTS
int main() { return frc::StartRobot<Robot>(); }
#endif
