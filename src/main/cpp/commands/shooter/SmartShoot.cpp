// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/shooter/SmartShoot.h"
#include "subsystems/Indexer.h"
#include "Constants.hpp"
#include <frc/smartdashboard/SmartDashboard.h>


// Shooter shoooter;
// Indexer indexing;

SmartShoot::SmartShoot(Shooter* _shooter, Indexer* _indexer, Intake* _intake, ArmSubsystem* _arm, double _shootSpeed,SwerveDrive* _swerdrive):
  shoooter{_shooter},
  indexing{_indexer},
  intake{_intake},
  arm{_arm},
  shootSpeed{_shootSpeed},
  swerdrive{_swerdrive}
  
{ 
  AddRequirements(indexing);
  AddRequirements(intake);
  AddRequirements(arm);
}

// Called when the command is initially scheduled.
void SmartShoot::Initialize() {
   
  // shootSpeed = std::min(std::fabs(frc::SmartDashboard::GetNumber("ARM_Speed",0.4)), 0.9);
  // shootAngle = frc::SmartDashboard::GetNumber("ARM_Angel",100);
  m_state = SMARTSPINUP;
}

// Called repeatedly when this Command is scheduled to run
void SmartShoot::Execute() { 
  //TODO: ADD CONSTANT FOR MOTOR SPEED CHECK
  switch (m_state)
  {
  case SMARTSPINUP:
  {
    shoooter->Shoot(shootSpeed);//angle is 78
    Arm_Position();
     if(shoooter->atSetpoint() && arm->m_ArmState == ArmConstants::DONE)
    {
      m_state = SMARTSHOOTING;
    }
    break;
  }
  case SMARTSHOOTING:
  {
    intake->intakeIndex();
    shoooter->Shoot(shootSpeed);
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

  m_state = SMARTDONE;
}

// Returns true when the command should end.
bool SmartShoot::IsFinished() {
  return false;
}

void SmartShoot::Arm_Position()
{  frc::Pose2d blue_Speaker_Pos = frc::Pose2d(0_m  , units::length::meter_t{5.583}, frc::Rotation2d{}); 
   auto currentPos = swerdrive->GetPose();
   auto robot2Speaker = currentPos.RelativeTo(blue_Speaker_Pos);
   float distance = sqrt(robot2Speaker.X().value()*robot2Speaker.X().value()+robot2Speaker.Y().value()*robot2Speaker.Y().value());
   frc::SmartDashboard::PutNumber("Distance",distance);
   //Make into feet
   distance = (distance+1.5) * 3.28084 ;
   int i = distance/3;
   switch(i)
   {
   case 0:

   frc::SmartDashboard::PutNumber("DIStance_TEST",i);
   arm->handle_Setpoint(units::angle::degree_t{40});
    break;
   case 1:
   frc::SmartDashboard::PutNumber("DIStance_TEST",i);
   arm->handle_Setpoint(units::angle::degree_t{50});
   break;
   case 2:
   frc::SmartDashboard::PutNumber("DIStance_TEST",i);
   arm->handle_Setpoint(units::angle::degree_t{57});
    break;
    case 3:
   frc::SmartDashboard::PutNumber("DIStance_TEST",i);
  //m_arm.handle_Setpoint(units::angle::degree_t{40});
    break;
    case 4:
   frc::SmartDashboard::PutNumber("DIStance_TEST",i);
  //m_arm.handle_Setpoint(units::angle::degree_t{40});
    break;
   
    default:
    frc::SmartDashboard::PutNumber("DIStance_TEST",i);
    arm->handle_Setpoint(units::angle::degree_t{45});  
    break;
    }

}
