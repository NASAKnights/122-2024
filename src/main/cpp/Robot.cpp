// Copyright (c) FRC Team 122. All Rights Reserved.

#include "Robot.hpp"


Robot::Robot() { this->CreateRobot(); }

void Robot::RobotInit(){
  std::string a1Name = "3NoteSpeakerRunB";
  auto a1 = pathplanner::PathPlannerAuto(a1Name);
  auto a1Pose = pathplanner::PathPlannerAuto::getPathGroupFromAutoFile(a1Name)[0]->getPathPoses()[0];
  auto entry1 = std::make_pair(std::move(a1),a1Pose);
  autoMap.emplace(0, std::move(entry1));

  std::string a2Name = "2NoteSpeakerRunB";
  auto a2 = pathplanner::PathPlannerAuto(a2Name);
  auto a2Pose = pathplanner::PathPlannerAuto::getPathGroupFromAutoFile(a2Name)[0]->getPathPoses()[0];
  auto entry2 = std::make_pair(std::move(a2),a2Pose);
  autoMap.emplace(1, std::move(entry2));

  std::string a3Name = "Note1FarAmpRun";
  auto a3 = pathplanner::PathPlannerAuto(a3Name);
  auto a3Pose = pathplanner::PathPlannerAuto::getPathGroupFromAutoFile(a3Name)[0]->getPathPoses()[0];
  auto entry3 = std::make_pair(std::move(a3),a3Pose);
  autoMap.emplace(2, std::move(entry3));

  std::string a4Name = "Note5FarAmpRun";
  auto a4 = pathplanner::PathPlannerAuto(a4Name);
  auto a4Pose = pathplanner::PathPlannerAuto::getPathGroupFromAutoFile(a4Name)[0]->getPathPoses()[0];
  auto entry4 = std::make_pair(std::move(a4),a4Pose);
  autoMap.emplace(3, std::move(entry4));

  std::string a5Name = "ShootB";
  auto a5 = pathplanner::PathPlannerAuto(a5Name);
  auto a5Pose = pathplanner::PathPlannerAuto::getPathGroupFromAutoFile(a5Name)[0]->getPathPoses()[0];
  auto entry5 = std::make_pair(std::move(a5),a5Pose);
  autoMap.emplace(4, std::move(entry5));

  std::string a6Name = "4NoteAutoButFast";
  auto a6 = pathplanner::PathPlannerAuto(a6Name);
  auto a6Pose = pathplanner::PathPlannerAuto::getPathGroupFromAutoFile(a6Name)[0]->getPathPoses()[0];
  auto entry6 = std::make_pair(std::move(a6),a6Pose);
  autoMap.emplace(5, std::move(entry6));

  std::string a7Name = "5NoteSteal";
  auto a7 = pathplanner::PathPlannerAuto(a7Name);
  auto a7Pose = pathplanner::PathPlannerAuto::getPathGroupFromAutoFile(a7Name)[0]->getPathPoses()[0];
  auto entry7 = std::make_pair(std::move(a7),a7Pose);
  autoMap.emplace(6, std::move(entry7));

  m_LED_Controller.DefaultAnimation();
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
  if (m_climber.atBot()) m_pdh.SetSwitchableChannel(false);
  else m_pdh.SetSwitchableChannel(true);
 
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
  // m_autonomousCommand = this->GetAutonomousCommand();
  m_swerveDrive.TurnVisionOff(); // don't use vision during Auto
  bool blue = autoColor.Get();
  int autonum = pow(2, 2) * autoSwitch6.Get() + 
                pow(2, 1) * autoSwitch7.Get() + 
                pow(2, 0) * autoSwitch8.Get();
  
  auto start = std::move(autoMap.at(autonum)).second;
   
  m_autonomousCommand = std::move(std::move(autoMap.at(autonum)).first).ToPtr();
  // frc::SmartDashboard::PutNumber("MoveRY",start.Y().value());
  // frc::SmartDashboard::PutNumber("MoveRX",start.X().value());
  if(!blue) start = pathplanner::GeometryUtil::flipFieldPose(start);
  m_swerveDrive.ResetPose(start);
  // auto a = std::move(autoMap[autonum].first);
  frc::SmartDashboard::PutString("Auto", std::move(autoMap.at(autonum)).first.GetName());

  if (m_autonomousCommand)
  {
    m_autonomousCommand->Schedule();
  }
}

void Robot::AutonomousPeriodic() {}

void Robot::TeleopInit()
{
  // m_led_control->m_intakeState;
  // This makes sure that the autonomous stops running when
  // teleop starts running. If you want the autonomous to
  // continue until interrupted by another command, remove
  // this line or comment it out.
  if (m_autonomousCommand)
  {
    m_autonomousCommand->Cancel();
  }
  m_swerveDrive.TurnVisionOn(); // Turn Vision back on for Teleop
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

  pathplanner::NamedCommands::registerCommand("a_shoot", std::move(Shoot(&m_shooter, &m_indexer, &m_intake, &m_arm, &m_LED_Controller, 0.7, 
                                              5, 0.75_s).ToPtr())); 
  pathplanner::NamedCommands::registerCommand("a_runIntake", std::move(IntakeNote(&m_intake, &m_indexer, &m_arm, &m_LED_Controller).ToPtr()));
  pathplanner::NamedCommands::registerCommand("a_armDown", std::move(ArmDown(&m_arm).ToPtr()));
  pathplanner::NamedCommands::registerCommand("a_farShot", std::move(Shoot(&m_shooter, &m_indexer, &m_intake, &m_arm, &m_LED_Controller, 0.8, 
                                              ArmConstants::kArmAngleShootFar, 1_s).ToPtr()));
  pathplanner::NamedCommands::registerCommand("a_stealShot", std::move(Shoot(&m_shooter, &m_indexer, &m_intake, &m_arm, &m_LED_Controller, 0.2, 
                                              ArmConstants::kArmAngleIntake, 1_s).ToPtr()));
  pathplanner::NamedCommands::registerCommand("a_knightShot", std::move(Shoot(&m_shooter, &m_indexer, &m_intake, &m_arm, &m_LED_Controller, 0.8, 
                                              ArmConstants::kArmAngleShootClose, 1_s).ToPtr()));

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

  frc2::JoystickButton(&m_operatorController, 1)
      .WhileTrue(Shoot(&m_shooter, &m_indexer, &m_intake, &m_arm, &m_LED_Controller,
                       0.3, 92, 1_s)
                     .ToPtr()); 
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
                       0.9, ArmConstants::kArmAngleShootFar, 2_s)
                     .ToPtr()); 

  frc2::POVButton(&m_operatorController, 90)
      .WhileTrue(Shoot(&m_shooter, &m_indexer, &m_intake, &m_arm, &m_LED_Controller,
                       0.9, ArmConstants::kArmAngleShootClose, 2_s)
                     .ToPtr());    
  
  /*frc2::POVButton(&m_operatorController, 180)
      .WhileTrue(Shoot(&m_shooter, &m_indexer, &m_intake, &m_arm, &m_LED_Controller,
                       0.4, ArmConstants::kArmAngleShootClose, 2_s)
                     .ToPtr());        
*/

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
  // auto start = pathplanner::PathPlannerAuto::getPathGroupFromAutoFile("MoveB")[0]->getPathPoses()[0];
  // frc::SmartDashboard::PutNumber("MoveRY",start.Y().value());
  // frc::SmartDashboard::PutNumber("MoveRX",start.X().value());
  // start = pathplanner::GeometryUtil::flipFieldPose(start);
  // m_swerveDrive.ResetPose(start);
              
  // autonomousToRun = std::move(autoMap[autonum]);

  // return autonomousToRun;


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
    m_LED_Controller.candle.SetLEDs(0, 0, 0, 0, numLED, 8-numLED);
  }
  else
  {
    m_LED_Controller.candle.SetLEDs(255, 0, 0, 0, 0, numLED);
    m_LED_Controller.candle.SetLEDs(0, 0, 0, 0, numLED, 8-numLED);
  }
}

/**'
 * 
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
  autoBlueStart1 = pathplanner::PathPlannerAuto::getPathGroupFromAutoFile("3NoteSpeakerRun")[0]->getPathPoses()[0];
  autoBlue1 = pathplanner::PathPlannerAuto("3NoteSpeakerRun").ToPtr();
}

#ifndef RUNNING_FRC_TESTS
int main() { return frc::StartRobot<Robot>(); }
#endif
