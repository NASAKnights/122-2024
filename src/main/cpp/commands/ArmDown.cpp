// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/ArmDown.h"

ArmDown::ArmDown(ArmSubsystem* _arm) :
        m_arm{_arm}
{
  // Use addRequirements() here to declare subsystem dependencies.
  AddRequirements(m_arm);
}

// Called when the command is initially scheduled.
void ArmDown::Initialize() {}

// Called repeatedly when this Command is scheduled to run
void ArmDown::Execute() {
  m_arm->handle_Setpoint(units::angle::degree_t{0.0});

}

// Called once the command ends or is interrupted.
void ArmDown::End(bool interrupted) {}

// Returns true when the command should end.
bool ArmDown::IsFinished() {
  return false;
}
