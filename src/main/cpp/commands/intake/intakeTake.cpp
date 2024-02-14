// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/intake/intakeTake.h"
#include "subsystems/Intake.h"
#include "subsystems/Shooter.h"

// Intake intake;
// Shooter shooter;

intakeTake::intakeTake(Intake* _intake, Indexer* _indexer):
  intake{_intake},
  indexer{_indexer}
{
  // Use addRequirements() here to declare subsystem dependencies.
  // intake = _intake;
  // shooter = _shoot;
}

// Called when the command is initially scheduled.
void intakeTake::Initialize() {}

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
void intakeTake::End(bool interrupted) { intake->stopIntake(); }

// Returns true when the command should end.
bool intakeTake::IsFinished() {
  return false;
}
