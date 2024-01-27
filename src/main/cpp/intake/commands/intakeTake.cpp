// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "intake/commands/intakeTake.h"
#include "intake/Intake.h"

Intake intake;

intakeTake::intakeTake(Intake* _intake):
  intake{_intake}
{
  // Use addRequirements() here to declare subsystem dependencies.
  intake = _intake;
}

// Called when the command is initially scheduled.
void intakeTake::Initialize() {}

// Called repeatedly when this Command is scheduled to run
void intakeTake::Execute() { intake->runIntake(); }

// Called once the command ends or is interrupted.
void intakeTake::End(bool interrupted) { intake->stopIntake(); }

// Returns true when the command should end.
bool intakeTake::IsFinished() {
  return false;
}
