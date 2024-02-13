// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "arm/commands/GoToAngle.h"

GoToAngle::GoToAngle(Arm* i_arm, double angle): 
  m_arm(i_arm), 
  m_angle(angle)
{
  // Use addRequirements() here to declare subsystem dependencies.
  AddRequirements(m_arm);
  
}

// Called when the command is initially scheduled.
void GoToAngle::Initialize() {}

// Called repeatedly when this Command is scheduled to run
void GoToAngle::Execute() {
  m_arm->set_Arm_Position(m_angle);
}

// Called once the command ends or is interrupted.
void GoToAngle::End(bool interrupted) {}

// Returns true when the command should end.
bool GoToAngle::IsFinished() {
  return m_arm->atSetpoint();
}
