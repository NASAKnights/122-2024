// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/shooter/shoot.h"
#include "subsystems/Shooter.h"
#include "subsystems/Indexer.h"
#include "Constants.hpp"

// Shooter shoooter;
// Indexer indexing;

Shoot::Shoot(Shooter* _shooter, Indexer* _indexer, Intake* _intake, ArmSubsystem* _arm, double _shootSpeed, double _shootAngle):
  shoooter{_shooter},
  indexing{_indexer},
  intake{_intake},
  arm{_arm},
  shootSpeed{_shootSpeed},
  shootAngle{_shootAngle}
{ 
  AddRequirements(indexing);
  AddRequirements(intake);
  AddRequirements(arm);
}

// Called when the command is initially scheduled.
void Shoot::Initialize() {
  m_state = SPINUP;
}

// Called repeatedly when this Command is scheduled to run
void Shoot::Execute() { 
  //TODO: ADD CONSTANT FOR MOTOR SPEED CHECK
  switch (m_state)
  {
  case SPINUP:
  {
    shoooter->Shoot(shootSpeed);//angle is 78
    arm->handle_Setpoint(units::angle::degree_t(shootAngle));
    if(shoooter->atSetpoint() && arm->GetController().AtGoal())
    {
      m_state = SHOOTING;
    }
    break;
  }
  case SHOOTING:
  {
    intake->intakeIndex();
    shoooter->Shoot(shootSpeed);
    break;
  }
  default:
  {
    break;
  }
  }

}

// Called once the command ends or is interrupted.
void Shoot::End(bool interrupted) 
{ 
  shoooter->stopShooter();
  intake->stopIntake();

  m_state = DONE;
}

// Returns true when the command should end.
bool Shoot::IsFinished() {
  return false;
}
