// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/shooter/SmartShoot.h"
#include "subsystems/Indexer.h"
#include "Constants.hpp"
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/MathUtil.h>

// Shooter shoooter;
// Indexer indexing;

SmartShoot::SmartShoot(Shooter *_shooter,
                       Indexer *_indexer,
                       Intake *_intake,
                       ArmSubsystem *_arm,
                       double _shootSpeed,
                       SwerveDrive *_swerdrive,
                       frc::Joystick *_operatorController,
                       frc::Joystick *_driverController,
                       frc::DigitalInput* _color) :
          shoooter{_shooter},
          indexing{_indexer},
          intake{_intake},
          arm{_arm},
          shootSpeed{_shootSpeed},
          swerdrive{_swerdrive},
          operatorController{_operatorController},
          driverController{_driverController},
          color{_color}

{
  AddRequirements(indexing);
  AddRequirements(intake);
  AddRequirements(arm);
}

// Called when the command is initially scheduled.
void SmartShoot::Initialize()
{
  swerdrive->DisableDrive();
  spinupTime.Reset();
  spinupTime.Start();
  
  m_state = SMARTSPINUP;
}

// Called repeatedly when this Command is scheduled to run
void SmartShoot::Execute()
{
  // TODO: ADD CONSTANT FOR MOTOR SPEED CHECK
  // shoooter->Shoot(shootSpeed);//angle is 78
   switch (m_state)
  {
    case SMARTSPINUP:
    { 
      Arm_Position();
      shoooter->Shoot(shootSpeed); 
      if(operatorController->GetRawButton(6))
      {
       if (arm->m_ArmState == ArmConstants::DONE && swerdrive->atSetpoint() )
       {
         m_state = SMARTSHOOTING;
       }
      }
      break;
    }
    case SMARTSHOOTING:
    {
      intake->intakeIndex();
      break;
    }
    default:
    {
      break;
    }
  }
}
// Called once the command ends or is interrupted.
void SmartShoot::End(bool interrupted)
{
  shoooter->stopShooter();
  intake->stopIntake();
  swerdrive->EnableDrive();
  m_state = SMARTDONE;
}

// Returns true when the command should end.
bool SmartShoot::IsFinished()
{
  return false;
}

void SmartShoot::Arm_Position()
{
  frc::SmartDashboard::PutBoolean("Color?", color->Get());
  frc::Pose2d blue_Speaker_Pos = frc::Pose2d(0_m, units::length::meter_t{5.583}, frc::Rotation2d{});
  frc::Pose2d red_Speaker_Pos = frc::Pose2d(units::length::meter_t{16.56}, units::length::meter_t{5.583}, frc::Rotation2d{});

  auto leftXAxis =
      MathUtilNK::calculateAxis(driverController->GetRawAxis(1),
                                DriveConstants::kDefaultAxisDeadband);
  auto leftYAxis =
      MathUtilNK::calculateAxis(driverController->GetRawAxis(0),
                                DriveConstants::kDefaultAxisDeadband);
  auto rightXAxis =
      MathUtilNK::calculateAxis(driverController->GetRawAxis(2),
                                DriveConstants::kDefaultAxisDeadband);
  auto speed =
      (frc::ChassisSpeeds::FromFieldRelativeSpeeds(
          -leftXAxis * DriveConstants::kMaxTranslationalVelocity,
          -leftYAxis * DriveConstants::kMaxTranslationalVelocity,
          -rightXAxis * DriveConstants::kMaxRotationalVelocity,
          swerdrive->GetHeading()));

  if (color->Get()) // true = blue, false = red
  { 
    auto currentPos = swerdrive->GetPose();
    auto robot2Speaker = currentPos.RelativeTo(blue_Speaker_Pos);
    distance = sqrt(robot2Speaker.X().value() * robot2Speaker.X().value() + 
                    robot2Speaker.Y().value() * robot2Speaker.Y().value());
    float angle = (asin(robot2Speaker.Y().value() / distance));

    swerdrive->Strafe(speed, angle);
    frc::SmartDashboard::PutNumber("rotataion",angle);
  }
  else
  {
    auto currentPos = swerdrive->GetPose();
    auto robot2Speaker = currentPos.RelativeTo(red_Speaker_Pos);

    distance = sqrt(robot2Speaker.X().value() * robot2Speaker.X().value() + 
                    robot2Speaker.Y().value() * robot2Speaker.Y().value());

    float angle = (asin(robot2Speaker.Y().value() / distance));

    swerdrive->Strafe(speed, -angle - M_PI);
    frc::SmartDashboard::PutNumber("rotataion", -angle - M_PI);
  }

  // Make into feet
  distance = ((distance) * 3.28084) - 4;
  int i = std::round(distance);
  frc::SmartDashboard::PutNumber("Distance_TEST", i);
  switch (i)
  {
  case 0:
    arm->handle_Setpoint(units::angle::degree_t{0.0});
    break;
  case 1:
    arm->handle_Setpoint(units::angle::degree_t{3});
    break;
  case 2:
    arm->handle_Setpoint(units::angle::degree_t{7});
    break;
  case 3:
    arm->handle_Setpoint(units::angle::degree_t{17.5});
    break;
  case 4:
    arm->handle_Setpoint(units::angle::degree_t{11});
    break;
  case 5:
    arm->handle_Setpoint(units::angle::degree_t{11});
    break;
  case 6:
    arm->handle_Setpoint(units::angle::degree_t{22});
    break;
  case 7:
    arm->handle_Setpoint(units::angle::degree_t{17.5});
    break;
  case 8:
    arm->handle_Setpoint(units::angle::degree_t{19});
    break;
  case 9:
    arm->handle_Setpoint(units::angle::degree_t{21});
    break;
  case 10:
    arm->handle_Setpoint(units::angle::degree_t{22});
    break;
  case 11:
    arm->handle_Setpoint(units::angle::degree_t{23});
    break;
  case 12:
    arm->handle_Setpoint(units::angle::degree_t{24});
    break;
  case 13:
    arm->handle_Setpoint(units::angle::degree_t{26});
    break;
  case 14:
    arm->handle_Setpoint(units::angle::degree_t{28});
    break;
  case 15:
    arm->handle_Setpoint(units::angle::degree_t{29});
    break;
  case 16:
    arm->handle_Setpoint(units::angle::degree_t{30});
    break;
  case 17:
    arm->handle_Setpoint(units::angle::degree_t{30.5});
    break;
  case 18:
    arm->handle_Setpoint(units::angle::degree_t{31.5});
    break;
  case 19:
    arm->handle_Setpoint(units::angle::degree_t{32});
    break;
  case 20:
    arm->handle_Setpoint(units::angle::degree_t{33});
    break;
  case 21:
    arm->handle_Setpoint(units::angle::degree_t{33});
    break;
  default:
    break;
  }
}
