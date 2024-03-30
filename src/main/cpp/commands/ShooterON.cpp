// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/ShooterON.h"

ShooterON::ShooterON(Shooter* _shooter): 
  shooter{_shooter}
{
  // Use addRequirements() here to declare subsystem dependencies.
}

// Called when the command is initially scheduled.
void ShooterON::Initialize() {
  
}

// Called repeatedly when this Command is scheduled to run
void ShooterON::Execute() {
  shooter->Shoot(0.75);
}

// Called once the command ends or is interrupted.
void ShooterON::End(bool interrupted) {
  shooter->stopShooter();
}

// Returns true when the command should end.
bool ShooterON::IsFinished() {
  return false;
}
