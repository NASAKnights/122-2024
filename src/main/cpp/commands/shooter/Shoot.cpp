// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/shooter/shoot.h"
#include "subsystems/Shooter.h"
#include "subsystems/Indexer.h"
#include "Constants.hpp"

// Shooter shoooter;
// Indexer indexing;


double rampSpeedDeadzone = 10;
double rampSpeedLower = ShooterConstants::motorRampSpeed - rampSpeedDeadzone;

Shoot::Shoot(Shooter* _shooter, Indexer* _indexer, Intake* _intake):
  shoooter{_shooter},
  indexing{_indexer},
  intake{_intake}
{}

// Called when the command is initially scheduled.
void Shoot::Initialize() {
  m_state = SPINUP;
}

// Called repeatedly when this Command is scheduled to run
void Shoot::Execute() { 
  //TODO: ADD CONSTANT FOR MOTOR SPEED CHECK
  switch (m_state)
  {
  case SPINUP:
  {
    shoooter->Shoot();
    if(fabs(shoooter->getSpeed()) >= ((5600.0/60.0) * 0.6))
    {
      m_state = SHOOTING;
    }
    break;
  }
  case SHOOTING:
  {
    intake->intakeIndex();
    shoooter->Shoot();
    break;
  }
  default:
  {
    break;
  }
  }

}

// Called once the command ends or is interrupted.
void Shoot::End(bool interrupted) 
{ 
  shoooter->stopShooter();
  intake->stopIntake();

  m_state = DONE;
}

// Returns true when the command should end.
bool Shoot::IsFinished() {
  return false;
}
