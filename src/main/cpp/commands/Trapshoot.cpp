// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/Trapshoot.h"


Trapshoot::Trapshoot(Shooter* _shooter, Indexer* _indexer,  Intake* _intake, ArmSubsystem* _arm, 
          double _shootSpeed, double _shootAngle, units::second_t _spinupTime = 1_s,double _diff = 1) : 
      shoooter{_shooter},
      indexing{_indexer},
      intake{_intake},
      arm{_arm},
      shootSpeed{_shootSpeed},
      shootAngle{_shootAngle},
      shooterSpinupTime{_spinupTime},
      diff{_diff}

{ 
  AddRequirements(indexing);
  AddRequirements(intake);
  AddRequirements(arm);
}

// Called when the command is initially scheduled.
void Trapshoot::Initialize() {
  spinupTime.Reset();
  spinupTime.Start();
  m_state = TrapSPINUP;
}

// Called repeatedly when this Command is scheduled to run
void Trapshoot::Execute() { 
  switch (m_state)
  {
    case TrapSPINUP:
    {
      shoooter->TrapShoot(shootSpeed,diff);//angle is 78
      arm->handle_Setpoint(units::angle::degree_t(shootAngle));
      
      if (spinupTime.HasElapsed(shooterSpinupTime) && arm->m_ArmState == ArmConstants::DONE)
      {
        m_state = TrapSHOOTING;
      }
      break;
    }
    case TrapSHOOTING:
    {
      arm->handle_Setpoint(units::angle::degree_t(shootAngle));
      if (shoooter->atSetpoint() || shootSpeed < 0.5) {
        intake->intakeIndex();
      }
      else {
        intake->stopIntake();
      }
      shoooter->TrapShoot(shootSpeed,diff);//angle is 78
      break;
    }
    default:
    {
      break;
    }
  }
}

// Called once the command ends or is interrupted.
void Trapshoot::End(bool interrupted) 
{ 
  shoooter->stopShooter();
  intake->stopIntake();
  m_state = TrapDONE;
}

// Returns true when the command should end.
bool Trapshoot::IsFinished() {
  return false;
}
