// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/Extend.h"
#include <subsystems/Climber.h>


Extend::Extend(Climber* _climber):
    climber{_climber}
{

}

// Called when the command is initially scheduled.
void Extend::Initialize() {}

// Called repeatedly when this Command is scheduled to run
void Extend::Execute() {


  climber->extend();
}

// Called once the command ends or is interrupted.
void Extend::End(bool interrupted) {}

// Returns true when the command should end.
bool Extend::IsFinished() {
  return false;
}
