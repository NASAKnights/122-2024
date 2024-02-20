// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/shooter/NewShoot.h"
#include "subsystems/Shooter.h"
#include "subsystems/Indexer.h"
#include "Constants.hpp"

// Shooter shoooter;
// Indexer indexing;

// double rampSpeedDeadzone = 10;
// double rampSpeedLower = ShooterConstants::motorRampSpeed - rampSpeedDeadzone;

NewShoot::NewShoot(Shooter* _shooter, Indexer* _indexer, Intake* _intake, ArmSubsystem* _arm):
  shoooter{_shooter},
  indexing{_indexer},
  intake{_intake},
  arm{_arm}
{}

// Called when the command is initially scheduled.
void NewShoot::Initialize() {
  m_state = SPINUP;

  // targetPose = 
}

bool NewShoot::ValidateState(ArmSubsystem* _arm, Shooter* chuter, SwerveDrive* swerve) {
  if(chuter->getSpeed() < fabs(((5600.0/60.0) * 0.6))) {
    return false;
  }
  if(_arm->GetController().GetPositionError().value() > 5){
    return false;
  }
  if(fabs((swerve->GetPose()-targetPose).Rotation().Degrees().value()) > FieldConstants::angleTolerance.Degrees().value()){
    return false;
  }
}

// Called repeatedly when this Command is scheduled to run
void NewShoot::Execute() { 
  //TODO: ADD CONSTANT FOR MOTOR SPEED CHECK
  switch (m_state)
  {
  case SPINUP:
  {
    shoooter->Shoot();
    if(fabs(shoooter->getSpeed()) >= ((5600.0/60.0) * 0.6)
      && fabs(arm->GetController().GetPositionError().value()) < 5)
    {
      m_state = SHOOTING;
    }
    break;
  }
  case SHOOTING:
  {

    if(ValidateState(arm, shoooter)){
      intake->intakeIndex();
      shoooter->Shoot();
    }
    break;
  }
  default:
  {
    break;
  }
  }
}

// Called once the command ends or is interrupted.
void NewShoot::End(bool interrupted) 
{ 
  shoooter->stopShooter();
  intake->stopIntake();

  m_state = DONE;
}

// Returns true when the command should end.
bool NewShoot::IsFinished() {
  return false;
}
