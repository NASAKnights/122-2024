// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "arm/commands/ArmIn.h"
#include "arm/subsystem/Arm.h"

ArmIn::ArmIn(Arm *arm) {
  // Use addRequirements() here to declare subsystem dependencies.
}

// Called when the command is initially scheduled.
void ArmIn::Initialize() {}

// Called repeatedly when this Command is scheduled to run
void ArmIn::Execute() {}

// Called once the command ends or is interrupted.
void ArmIn::End(bool interrupted) {}

// Returns true when the command should end.
bool ArmIn::IsFinished() {
  return false;
}
