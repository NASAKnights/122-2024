// Copyright (c) FRC Team 122. All Rights Reserved.

#include "Robot.hpp"


Robot::Robot() { this->CreateRobot(); }

void Robot::RobotInit(){

};

/**
 * This function is called every 20 ms, no matter the mode. Use
 * this for items like diagnostics that you want to run during disabled,
 * autonomous, teleoperated and test.
 *
 * <p> This runs after the mode specific periodic functions, but before
 * LiveWindow and SmartDashboard integrated updating.
 */
void Robot::RobotPeriodic()
{
  frc2::CommandScheduler::GetInstance().Run();
  this->UpdateDashboard();
  m_arm.Emergency_Stop(); //check if arm has triggered a stop
}

/**
 * This function is called once each time the robot enters Disabled mode. You
 * can use it to reset any subsystem information you want to clear when the
 * robot is disabled.
 */
void Robot::DisabledInit()
{
  m_LED_Controller.DefaultAnimation();
}

/**
 * This autonomous runs the autonomous command selected by your {@link
 * RobotContainer} class.
 */
void Robot::AutonomousInit()
{
  m_autonomousCommand = this->GetAutonomousCommand();

  if (m_autonomousCommand)
  {
    m_autonomousCommand->Schedule();
  }
}

void Robot::AutonomousPeriodic() {}

void Robot::TeleopInit()
{
  // This makes sure that the autonomous stops running when
  // teleop starts running. If you want the autonomous to
  // continue until interrupted by another command, remove
  // this line or comment it out.
  if (m_autonomousCommand)
  {
    m_autonomousCommand->Cancel();
  }
  // m_LED_Controller.candle.ClearAnimation(0);
}

/**
 * This function is called periodically during operator control.
 */
void Robot::TeleopPeriodic()
{
  // m_arm.Periodic();
}

void Robot::TeleopExit()
{
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
void Robot::CreateRobot()
{
  // Initialize all of your commands and subsystems here
  frc::SmartDashboard::PutNumber("ARM_Angel", ArmConstants::kArmAngleDriving);
  // frc::SmartDashboard::PutNumber("ARM_Speed", -120);

  pathplanner::NamedCommands::registerCommand("a_shoot", std::move(Shoot(&m_shooter, &m_indexer, &m_intake, &m_arm, &m_LED_Controller, 0.8, 
                                              ArmConstants::kArmAngleShootClose).ToPtr())); 
  pathplanner::NamedCommands::registerCommand("a_runIntake", std::move(IntakeNote(&m_intake, &m_indexer, &m_arm, &m_LED_Controller).ToPtr()));
  pathplanner::NamedCommands::registerCommand("a_ArmDown", std::move(ArmDown(&m_arm).ToPtr()));

  m_swerveDrive.SetDefaultCommand(frc2::RunCommand(
      [this]
      {
        auto leftXAxis =
            MathUtilNK::calculateAxis(m_driverController.GetRawAxis(1),
                                      DriveConstants::kDefaultAxisDeadband);
        auto leftYAxis =
            MathUtilNK::calculateAxis(m_driverController.GetRawAxis(0),
                                      DriveConstants::kDefaultAxisDeadband);
        auto rightXAxis =
            MathUtilNK::calculateAxis(m_driverController.GetRawAxis(2),
                                      DriveConstants::kDefaultAxisDeadband);

        m_swerveDrive.Drive(frc::ChassisSpeeds::FromFieldRelativeSpeeds(
            -leftXAxis * DriveConstants::kMaxTranslationalVelocity,
            -leftYAxis * DriveConstants::kMaxTranslationalVelocity,
            -rightXAxis * DriveConstants::kMaxRotationalVelocity,
            m_swerveDrive.GetHeading()));
      },
      {&m_swerveDrive}));

  m_arm.SetDefaultCommand(frc2::RunCommand(
      [this]
      {
        m_arm.handle_Setpoint(units::degree_t(ARM_Angel));
      },
      {&m_arm}));

  // Configure the button bindings
  BindCommands();
  loadAutonomous();
  m_swerveDrive.ResetHeading();
  AddPeriodic([this]
              { m_arm.Periodic(); },
              5_ms, 1_ms);
}

/**
 * Binds commands to Joystick buttons
 */
void Robot::BindCommands()
{

//---------------DRIVER BUTTONS----------------------------------
  frc2::JoystickButton(&m_driverController, 1)
      .OnTrue(frc2::CommandPtr(frc2::InstantCommand([this]
                                                    { return m_swerveDrive.ResetHeading(); }))); // TODO assign as test

  frc2::JoystickButton(&m_driverController, 6)
      .OnTrue(frc2::InstantCommand([this]
          { frc::SmartDashboard::PutNumber("ARM_Angel", 0.0); }).ToPtr())
      .OnFalse(frc2::CommandPtr(frc2::InstantCommand([this]
          { frc::SmartDashboard::PutNumber("ARM_Angel", ArmConstants::kArmAngleDriving); }).ToPtr()));


  frc2::JoystickButton(&m_driverController, 9)
                          .OnTrue(frc2::InstantCommand([this]
                            {frc::SmartDashboard::PutNumber("ARM_Angel", ArmConstants::kArmAngleStarting); }).ToPtr())
                          .OnFalse(frc2::CommandPtr(frc2::InstantCommand([this]
                            {frc::SmartDashboard::PutNumber("ARM_Angel", ArmConstants::kArmAngleDriving); }).ToPtr()));

  frc2::JoystickButton(&m_driverController, 10)
      .OnTrue(frc2::RunCommand([this]{m_climber.retractLimit_Pit();},{&m_climber}).ToPtr())
      .OnFalse(frc2::InstantCommand([this] {m_climber.stopMotor();},{&m_climber}).ToPtr());

// --------------OPERATOR BUTTONS--------------------------------
  
  // frc2::JoystickButton(&m_operatorController, 1)
      // .WhileTrue(Shoot(&m_shooter, &m_indexer, &m_intake, &m_arm, &m_LED_Controller,
                      //  0.8, ArmConstants::kArmAngleShootFar).ToPtr()); //Far Shot

  
  frc2::JoystickButton(&m_operatorController, 3)
      .WhileTrue(IntakeNote(&m_intake, &m_indexer, &m_arm, &m_LED_Controller).ToPtr());

  frc2::JoystickButton(&m_operatorController, 4)
      .OnTrue(frc2::RunCommand([this]
          { m_intake.runIntakeReverse(); },
          {&m_intake}).ToPtr())
      .OnFalse(frc2::InstantCommand([this]
          { return m_intake.stopIntake(); },
          {&m_intake}).ToPtr());

  frc2::JoystickButton(&m_operatorController, 8)
      .WhileTrue(SmartShoot(&m_shooter, &m_indexer, &m_intake, &m_arm, 
                    0.85, &m_swerveDrive, &m_operatorController, &m_driverController, &autoColor).ToPtr());

  // frc2::JoystickButton(&m_operatorController, 6)
      // .WhileTrue(Shoot(&m_shooter, &m_indexer, &m_intake, &m_arm, 
                      // &m_LED_Controller, 0.9, ArmConstants::kArmAngleShootClose).ToPtr());

  frc2::POVButton(&m_operatorController, 270)
      .WhileTrue(Shoot(&m_shooter, &m_indexer, &m_intake, &m_arm, &m_LED_Controller,
                       0.9, ArmConstants::kArmAngleShootFar)
                     .ToPtr()); 

  frc2::POVButton(&m_operatorController, 90)
      .WhileTrue(Shoot(&m_shooter, &m_indexer, &m_intake, &m_arm, &m_LED_Controller,
                       0.9, ArmConstants::kArmAngleShootClose)
                     .ToPtr());        


  /* frc2::JoystickButton(&m_operatorController, 6)
              .WhileTrue(SmartShoot(&m_shooter, &m_indexer, &m_intake, &m_arm,0.8,&m_swerveDrive,&m_operatorController,&m_driverController,&autoColor).ToPtr());

     */

  frc2::JoystickButton(&m_operatorController, 9)
    .WhileTrue(Retract(&m_climber).ToPtr());

  frc2::JoystickButton(&m_operatorController, 10)
    .WhileTrue(Extend(&m_climber).ToPtr());



  
}
/**
 * Returns the Autonomous Command
 */
frc2::CommandPtr Robot::GetAutonomousCommand()
{

  auto start = pathplanner::PathPlannerAuto::getPathGroupFromAutoFile("MoveB")[0]->getPathPoses()[0];
  frc::SmartDashboard::PutNumber("MoveRY",start.Y().value());
  frc::SmartDashboard::PutNumber("MoveRX",start.X().value());
  start = pathplanner::GeometryUtil::flipFieldPose(start);
  m_swerveDrive.ResetPose(start);
  return pathplanner::PathPlannerAuto("MoveB").ToPtr();

  int autonum = pow(2, 3) * autoColor.Get() + 
                pow(2, 2) * autoSwitch6.Get() + 
                pow(2, 1) * autoSwitch7.Get() + 
                pow(2, 0) * autoSwitch8.Get();

  // auto start = pathplanner::PathPlannerAuto::getPathGroupFromAutoFile("ShootDriveB")[0]->getPathPoses()[0];
  // m_swerveDrive.ResetPose(start);

  // return pathplanner::PathPlannerAuto("ShootDriveB").ToPtr();
  /*if(autoColor.Get())
  { if(autoSwitch8.Get()){
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
    if(autoSwitch8.Get()){
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
  }*/
}

void Robot::DisabledPeriodic()
{
  int numLED = pow(2, 2) * autoSwitch6.Get() + 
                pow(2, 1) * autoSwitch7.Get() + 
                pow(2, 0) * autoSwitch8.Get() + 1;
  if (autoColor.Get())
  {
    m_LED_Controller.candle.SetLEDs(0, 0, 255, 0, 0, numLED);
    m_LED_Controller.candle.SetLEDs(0, 0, 0, 0, numLED, 7-numLED);
  }
  else
  {
    m_LED_Controller.candle.SetLEDs(255, 0, 0, 0, 0, numLED);
    m_LED_Controller.candle.SetLEDs(0, 0, 0, 0, numLED, 7-numLED);
  }
  frc::SmartDashboard::PutNumber("Auto", autoName);
}

/**
 * Updates the data on the dashboard
 */
void Robot::UpdateDashboard()
{
  // frc::SmartDashboard::PutNumber("driver X", m_driverController.GetX());
  // frc::SmartDashboard::PutNumber(
  //     "adjusted X",
  //     MathUtilNK::calculateAxis(m_driverController.GetX(),
  //                               DriveConstants::kDefaultAxisDeadband) *
  //         DriveConstants::kMaxTranslationalVelocity.value());
  // frc::SmartDashboard::PutNumber("Swerve Drive Heading",
  //                                m_swerveDrive.GetHeading().Degrees().value());
  ARM_Angel = frc::SmartDashboard::GetNumber("ARM_Angel", ArmConstants::kArmAngleDriving);
  // ARM_Speed = frc::SmartDashboard::GetNumber("ARM_Speed", -120);
  servo_angle = frc::SmartDashboard::GetNumber("servo_angle", 100);
  frc::SmartDashboard::PutBoolean("AutoSwitches/color", autoColor.Get());
  frc::SmartDashboard::PutBoolean("AutoSwitches/6", autoSwitch6.Get());
  frc::SmartDashboard::PutBoolean("AutoSwitches/7", autoSwitch7.Get());
  frc::SmartDashboard::PutBoolean("AutoSwitches/8", autoSwitch8.Get());

  frc::SmartDashboard::PutBoolean("EmergencySwitch4",m_arm.isOverLimit());

  m_arm.printLog();
}

void Robot::loadAutonomous() {
  autoBlueStart1 = pathplanner::PathPlannerAuto::getPathGroupFromAutoFile("MoveB")[0]->getPathPoses()[0];
  autoBlue1 = pathplanner::PathPlannerAuto("MoveB").ToPtr();



}

#ifndef RUNNING_FRC_TESTS
int main() { return frc::StartRobot<Robot>(); }
#endif
