// Copyright (c) FRC Team 122. All Rights Reserved.

#include "Robot.hpp"

#include <frc/smartdashboard/SmartDashboard.h>
#include <frc2/command/CommandScheduler.h>
#include <frc2/command/RunCommand.h>
#include <frc2/command/button/JoystickButton.h>
#include <units/angular_velocity.h>
#include <units/velocity.h>

#include <pathplanner/lib/auto/AutoBuilder.h>
#include <pathplanner/lib/util/HolonomicPathFollowerConfig.h>
#include <pathplanner/lib/auto/NamedCommands.h>

#include "commands/TrajectoryFollower.hpp"
#include "frc2/command/InstantCommand.h"
#include "util/NKTrajectoryManager.hpp"
#include <arm/Arm.h>
#include <autos/Auto.h>
#include <cmath>
#include <commands/shooter/SmartShoot.h>


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
  m_LED_Controller.DefaultAnimation();
}


/**
 * This autonomous runs the autonomous command selected by your {@link
 * RobotContainer} class.
 */
void Robot::AutonomousInit() {
  // m_autonomousCommand = this->GetAutonomousCommand();

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
  m_LED_Controller.candle.ClearAnimation(0);
}

/**
 * This function is called periodically during operator control.
 */
void Robot::TeleopPeriodic() {
  // m_arm.Periodic();

}

void Robot::TeleopExit() {
  m_arm.arm_Brake_Out();
  m_climber.engage();
  m_climber.disableBrake();
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
      frc::SmartDashboard::PutNumber("ARM_Angel",ArmConstants::kArmAngleDriving);
      frc::SmartDashboard::PutNumber("ARM_Speed",-120);
  
      pathplanner::NamedCommands::registerCommand("a_shoot", std::move(Shoot(&m_shooter, &m_indexer, &m_intake, &m_arm, &m_LED_Controller, 0.8, ArmConstants::kArmAngleShootClose+1).ToPtr())); // <- This example method returns CommandPtr
      pathplanner::NamedCommands::registerCommand("a_runIntake", std::move(intakeTake(&m_intake, &m_indexer, &m_arm, &m_LED_Controller).ToPtr()));

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
        m_arm.handle_Setpoint(units::degree_t(ARM_Angel));
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



      //TODO Remove later



  frc2::JoystickButton(&m_driverController, 9).OnTrue(frc2::InstantCommand(
      [this] {
        frc::SmartDashboard::PutNumber("ARM_Angel", ArmConstants::kArmAngleStarting);
      }).ToPtr()
      ).OnFalse(frc2::CommandPtr(frc2::InstantCommand([this] {
        frc::SmartDashboard::PutNumber("ARM_Angel", ArmConstants::kArmAngleDriving);
      }).ToPtr()));

  frc2::JoystickButton(&m_operatorController, 9)
    .WhileTrue(Retract(&m_climber).ToPtr());
  
  frc2::JoystickButton(&m_operatorController, 10)
        .WhileTrue(Extend(&m_climber).ToPtr());

  // frc2::JoystickButton(&m_operatorController, 6).OnFalse(frc2::CommandPtr(frc2::InstantCommablend([this] {
  //       // m_arm.SetGoal(units::degree_t(m_arm.GetMeasurement()));
  //       m_arm.Disable();
  //     },
  //     {&m_arm}).ToPtr()))
  //     .WhileTrue(Shoot(&m_shooter, &m_indexer, &m_intake, &m_arm, &m_LED_Controller, ARM_Speed, ARM_Angel).ToPtr());

  frc2::JoystickButton(&m_operatorController, 6)
      .WhileTrue(Shoot(&m_shooter, &m_indexer, &m_intake, &m_arm,  &m_LED_Controller,
                0.8, ArmConstants::kArmAngleShootClose).ToPtr()); 
     /* .WhileTrue(SmartShoot(&m_shooter, &m_indexer, &m_intake, &m_arm, 
                0.8, &m_swerveDrive).ToPtr()); 
*/
  frc2::JoystickButton(&m_operatorController, 5)
      .WhileTrue(Shoot(&m_shooter, &m_indexer, &m_intake, &m_arm, &m_LED_Controller,
                0.8, 62.5).ToPtr());


  frc2::JoystickButton(&m_operatorController, 1)
      .WhileTrue(Shoot(&m_shooter, &m_indexer, &m_intake, &m_arm, &m_LED_Controller,
                0.8, 66.25).ToPtr());
 
//AMP
  /*frc2::JoystickButton(&m_operatorController, 5)
      .WhileTrue(Shoot(&m_shooter, &m_indexer, &m_intake, &m_arm, &m_LED_Controller,
                0.4, 105).ToPtr()); // -15, 145
*/
    frc2::JoystickButton(&m_operatorController, 3)
      .WhileTrue(intakeTake(&m_intake, &m_indexer, &m_arm, &m_LED_Controller).ToPtr());

  frc2::JoystickButton(&m_operatorController, 4)
    .OnTrue(frc2::RunCommand([this] {
      m_intake.runIntakeReverse();
    },{&m_intake}).ToPtr())
    .OnFalse(frc2::InstantCommand([this] {
      return m_intake.stopIntake();
    }, {&m_intake}).ToPtr());


  frc2::JoystickButton(&m_driverController, 6).OnTrue(frc2::InstantCommand(
      [this] {
        frc::SmartDashboard::PutNumber("ARM_Angel", 0.0);
      }).ToPtr()
      ).OnFalse(frc2::CommandPtr(frc2::InstantCommand([this] {
        frc::SmartDashboard::PutNumber("ARM_Angel", ArmConstants::kArmAngleDriving);
      }).ToPtr()));

}
/**
 * Returns the Autonomous Command
 */
// frc2::CommandPtr Robot::GetAutonomousCommand() {
//   return TrajectoryFollower(&m_swerveDrive,
//                             &NKTrajectoryManager::GetTrajectory("Test"))
//       .ToPtr();
//   return Auto(&m_swerveDrive, &m_shooter, &m_indexer, &m_intake, &m_arm, 1)
//             .ToPtr();
// }
frc2::CommandPtr Robot::GetAutonomousCommand(){
  
  if(auto3.Get())
  { if(auto4Note.Get()){
    auto start = pathplanner::PathPlannerAuto::getPathGroupFromAutoFile("3NoteSpeakerRun")[0]->getPathPoses()[0];
    m_swerveDrive.ResetPose(start);
    return pathplanner::PathPlannerAuto("3NoteSpeakerRun").ToPtr();
    }
    else
    { 
       auto start = pathplanner::PathPlannerAuto::getPathGroupFromAutoFile("Move")[0]->getPathPoses()[0];
       m_swerveDrive.ResetPose(start);
       return pathplanner::PathPlannerAuto("Move").ToPtr();
    }
  }
  else{
    if(auto4Note.Get()){
      auto start = pathplanner::PathPlannerAuto::getPathGroupFromAutoFile("3NoteSpeakerRun")[0]->getPathPoses()[0];
      m_swerveDrive.ResetPose(start);
      return pathplanner::PathPlannerAuto("3NoteSpeakerRun").ToPtr();
    }
    else
    { 
      auto start = pathplanner::PathPlannerAuto::getPathGroupFromAutoFile("Move")[0]->getPathPoses()[0];
      m_swerveDrive.ResetPose(start);
      return pathplanner::PathPlannerAuto("Move").ToPtr();
    }
  }
}

void Robot::DisabledPeriodic() {
  // m_arm.Disable();
  if(auto3.Get()){
    m_LED_Controller.candle.SetLEDs(0,0,255,0,0,4);
  }
  else{
    m_LED_Controller.candle.SetLEDs(255,0,0,0,0,4);
  }
  if(auto4Note.Get()){
    autoName = 1;
    m_LED_Controller.candle.SetLEDs(0,255,0,0,4,4);
  }
  else{
    autoName = 2;
    m_LED_Controller.candle.SetLEDs(0,0,0,0,4,4);
  }
  
  // Set Auto commands
  // if(auto3.Get())
  // { if(auto4Note.Get()){
  //     auto start = pathplanner::PathPlannerAuto::getPathGroupFromAutoFile("3NoteSpeakerRun")[0]->getPathPoses()[0];
  //     m_swerveDrive.ResetPose(start);
  //     m_autonomousCommand =  pathplanner::PathPlannerAuto("3NoteSpeakerRun").ToPtr();
  //   }
  //   else
  //   { 
  //      auto start = pathplanner::PathPlannerAuto::getPathGroupFromAutoFile("Move")[0]->getPathPoses()[0];
  //      m_swerveDrive.ResetPose(start);
  //      m_autonomousCommand =  pathplanner::PathPlannerAuto("Move").ToPtr();
  //   }
  // }
  // else{
  //   if(auto4Note.Get()){
  //     auto start = pathplanner::PathPlannerAuto::getPathGroupFromAutoFile("3NoteSpeakerRun")[0]->getPathPoses()[0];
  //     m_swerveDrive.ResetPose(start);
  //     m_autonomousCommand =  pathplanner::PathPlannerAuto("3NoteSpeakerRun").ToPtr();
  //   }
  //   else
  //   { 
  //     auto start = pathplanner::PathPlannerAuto::getPathGroupFromAutoFile("Move")[0]->getPathPoses()[0];
  //     m_swerveDrive.ResetPose(start);
  //     m_autonomousCommand =  pathplanner::PathPlannerAuto("Move").ToPtr();
  //   }
  // }

  frc::SmartDashboard::PutNumber("Auto", autoName);
  // m_swerveDrive.SetVision();
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
   ARM_Angel = frc::SmartDashboard::GetNumber("ARM_Angel",ArmConstants::kArmAngleDriving);
   ARM_Speed = frc::SmartDashboard::GetNumber("ARM_Speed",-120);
   servo_angle = frc::SmartDashboard::GetNumber("servo_angle",100);

  frc::SmartDashboard::PutBoolean("AutoSwitches/color", autoColor.Get());
  frc::SmartDashboard::PutBoolean("AutoSwitches/7", auto2.Get());
  frc::SmartDashboard::PutBoolean("AutoSwitches/6", auto3.Get());
  frc::SmartDashboard::PutBoolean("AutoSwitches/8", auto4Note.Get());

  // m_swerveDrive.PrintNetworkTableValues();
  m_arm.printLog();
  
}



#ifndef RUNNING_FRC_TESTS
int main() { return frc::StartRobot<Robot>(); }
#endif
