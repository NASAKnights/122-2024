// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/intake/intakeTake.h"
#include "subsystems/Intake.h"
#include "subsystems/Shooter.h"

intakeTake::intakeTake(Intake* _intake, Indexer* _indexer, ArmSubsystem* _arm):
  intake{_intake},
  indexer{_indexer},
  m_arm{_arm}
{
  // Use addRequirements() here to declare subsystem dependencies.
  // intake = _intake;
  // shooter = _shoot;
  AddRequirements(indexer);
  AddRequirements(intake);
  AddRequirements(m_arm);
}

// Called when the command is initially scheduled.
void intakeTake::Initialize() {
  m_state = MOVING;
  m_arm->SetGoal(units::angle::degree_t(38.5));
  m_arm->Enable();
}

// Called repeatedly when this Command is scheduled to run
void intakeTake::Execute() { 
  if(indexer->hasNote()){
    m_state = IDLE;
  }
  switch (m_state) {
    case MOVING:
    {
      intake->runIntake();
      break;
    }
    case IDLE:
    {
      intake->stopIntake();
      break;
    }
  }
}

// Called once the command ends or is interrupted.
void intakeTake::End(bool interrupted) { 
  intake->stopIntake(); 
  m_state = IDLE;
}

// Returns true when the command should end.
bool intakeTake::IsFinished() {
  return false;
}
