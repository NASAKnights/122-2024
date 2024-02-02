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

shoot::shoot(Shooter* _shooter, Indexer* _indexer):
  shoooter{_shooter},
  indexing{_indexer}
{
  // Use addRequirements() here to declare subsystem dependencies.
  shoooter = _shooter;
  indexing = _indexer;
}

// Called when the command is initially scheduled.
void shoot::Initialize() {}

// Called repeatedly when this Command is scheduled to run
void shoot::Execute() { 
  //TODO: ADD CONSTANT FOR MOTOR SPEED CHECK
  if(indexing->getIndex()) {
    while(shoooter->getSpeed() < rampSpeedLower) { shoooter->shoot(); }
    shoooter->shoot();
  }
}

// Called once the command ends or is interrupted.
void shoot::End(bool interrupted) { shoooter->stopShooter(); }

// Returns true when the command should end.
bool shoot::IsFinished() {
  return false;
}
