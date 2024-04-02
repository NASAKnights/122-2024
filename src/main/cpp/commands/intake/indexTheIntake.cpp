// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/intake/indexTheIntake.h"

indexTheIntake::indexTheIntake(Intake* _intake) :
  // Use addRequirements() here to declare subsystem dependencies.
  intake{_intake}
{
  AddRequirements(intake);
}

// Called when the command is initially scheduled.
void indexTheIntake::Initialize() {}

// Called repeatedly when this Command is scheduled to run
void indexTheIntake::Execute() {
  intake->runIntake();
}

// Called once the command ends or is interrupted.
void indexTheIntake::End(bool interrupted) {
  intake->stopIntake();
}

// Returns true when the command should end.
bool indexTheIntake::IsFinished() {
  return false;
}
