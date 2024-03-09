// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/LED/LEDDefault.h"

LEDDefault::LEDDefault(LEDController* controller) :
m_LEDController(controller)
{
  AddRequirements(controller);
}

// Called when the command is initially scheduled.
void LEDDefault::Initialize() {
  m_LEDController->DefaultAnimation();
}

// Called repeatedly when this Command is scheduled to run
void LEDDefault::Execute() {}

// Called once the command ends or is interrupted.
void LEDDefault::End(bool interrupted) {}

// Returns true when the command should end.
bool LEDDefault::IsFinished() {
  return false;
}
