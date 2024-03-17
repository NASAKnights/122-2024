// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/intake/IntakeNote.h"


IntakeNote::IntakeNote(Intake* _intake, Indexer* _indexer, ArmSubsystem* _arm, LEDController* led_controller):
  intake{_intake},
  indexer{_indexer},
  m_arm{_arm},
  m_led_control{led_controller}
{
  // Use addRequirements() here to declare subsystem dependencies.
  // intake = _intake;
  // shooter = _shoot;
  AddRequirements(indexer);
  AddRequirements(intake);
  AddRequirements(m_arm);
  AddRequirements(m_led_control);
}

// Called when the command is initially scheduled.
void IntakeNote::Initialize() {
  m_state = MOVING;
  m_arm->handle_Setpoint(units::angle::degree_t(ArmConstants::kArmAngleIntake)); //38.5
  m_led_control->m_intakeState = NO_NOTE;
  m_led_control->HandleIntakeState();
}

// Called repeatedly when this Command is scheduled to run
void IntakeNote::Execute() { 
  if(indexer->hasNote()){
    m_state = IDLE;
  }
  m_arm->handle_Setpoint(units::angle::degree_t(ArmConstants::kArmAngleIntake));

  switch (m_state) {
    case MOVING:
    {
      intake->runIntake();
      m_led_control->m_intakeState = NO_NOTE;
      m_led_control->HandleIntakeState();
      break;
    }
    case IDLE:
    {
      intake->stopIntake();
      m_led_control->m_intakeState = NOTE;
      m_led_control->HandleIntakeState();
      break;
    }
    default:
    {
      m_led_control->HandleIntakeState();
      break;
    }
  }
}

// Called once the command ends or is interrupted.
void IntakeNote::End(bool interrupted) { 
  intake->stopIntake(); 
  m_state = IDLE;
  m_led_control->candle.SetLEDs(0,255,255,1,8);
}

// Returns true when the command should end.
bool IntakeNote::IsFinished() {
  return false;
}
