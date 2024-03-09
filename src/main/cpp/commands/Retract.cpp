// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/Retract.h"
#include <subsystems/Climber.h>


Retract::Retract(Climber* _climber):
    climber{_climber}
{
  AddRequirements(climber);
}

// Called when the command is initially scheduled.
void Retract::Initialize() {}

// Called repeatedly when this Command is scheduled to run
void Retract::Execute() {
  climber->retract();
}

// Called once the command ends or is interrupted.
void Retract::End(bool interrupted) {
  climber->stopMotor();
}

// Returns true when the command should end.
bool Retract::IsFinished() {
  return false;
}
