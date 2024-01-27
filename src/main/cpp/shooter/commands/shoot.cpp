// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "shooter/commands/shoot.h"
#include "shooter/Shooter.h"
#include "indexer/Indexer.h"

Shooter shoooter;
Indexer index;

shoot::shoot(Shooter* _shooter):
  shoooter{_shooter}
{
  // Use addRequirements() here to declare subsystem dependencies.
  shoooter = _shooter;
}

// Called when the command is initially scheduled.
void shoot::Initialize() {}

// Called repeatedly when this Command is scheduled to run
void shoot::Execute() { 
  //TODO: ADD DETECTION
  shoooter->shoot();
}

// Called once the command ends or is interrupted.
void shoot::End(bool interrupted) { shoooter->stopShooter(); }

// Returns true when the command should end.
bool shoot::IsFinished() {
  return false;
}
