// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/shooter/MotorRun.h"
#include "subsystems/Shooter.h"
#include "subsystems/Indexer.h"
#include "Constants.hpp"
#include <frc/smartdashboard/SmartDashboard.h>
#include "subsystems/LEDController.h"

MotorRun::MotorRun(Shooter* _shooter, double _shootSpeed, units::second_t _spinupTime = 1_s) : 
      shoooter{_shooter},
      shootSpeed{_shootSpeed},
      shooterSpinupTime{_spinupTime}
{ 
  AddRequirements(shoooter);
}

// Called when the command is initially scheduled.
void MotorRun::Initialize() {}

// Called repeatedly when this Command is scheduled to run
void MotorRun::Execute() {
  shoooter->Shoot(shootSpeed);
}

// Called once the command ends or is interrupted.
void MotorRun::End(bool interrupted) 
{ 
  shoooter->stopShooter();
}

// Returns true when the command should end.
bool MotorRun::IsFinished() {
  return false;
}
