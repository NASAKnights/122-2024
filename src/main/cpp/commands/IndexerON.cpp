// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/IndexerON.h"

IndexerON::IndexerON(Intake* _intake): 
  intake{_intake}
{
  // Use addRequirements() here to declare subsystem dependencies.
}

// Called when the command is initially scheduled.
void IndexerON::Initialize() {}

// Called repeatedly when this Command is scheduled to run
void IndexerON::Execute() {

  intake->intakeIndex();
}

// Called once the command ends or is interrupted.
void IndexerON::End(bool interrupted) {
  intake->stopIntake();
}

// Returns true when the command should end.
bool IndexerON::IsFinished() {
  return false;
}
