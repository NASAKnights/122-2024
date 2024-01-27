// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "intake/commands/intakeOut.h"
#include "intake/Intake.h"

Intake intake;

intakeOut::intakeOut(Intake* _intake):
  intake{_intake}
{
  // Use addRequirements() here to declare subsystem dependencies.
  intake = _intake;
}

// Called when the command is initially scheduled.
void intakeOut::Initialize() {}

// Called repeatedly when this Command is scheduled to run
void intakeOut::Execute() { intake->runIntakeReverse(); }

// Called once the command ends or is interrupted.
void intakeOut::End(bool interrupted) { intake->stopIntake(); }

// Returns true when the command should end.
bool intakeOut::IsFinished() {
  return false;
}
