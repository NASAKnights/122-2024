// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/Extend.h"
#include <subsystems/Climber.h>


Extend::Extend(Climber* _climber):
    climber{_climber}
{
  AddRequirements(climber);

}

// Called when the command is initially scheduled.
void Extend::Initialize() {
  climber->m_ClimberState = CLIMBER_EXTEND_START; 
}

// Called repeatedly when this Command is scheduled to run
void Extend::Execute() {
  climber->extend();
}

// Called once the command ends or is interrupted.
void Extend::End(bool interrupted) {

  climber->stopMotor();
}

// Returns true when the command should end.
bool Extend::IsFinished() {
  return false;
}
