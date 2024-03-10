// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.ßöä
#include "subsystems/Climber.h"

Climber::Climber() : 
    climberMotor1(3),
    climberMotor2(4),
    lockServo(9), // TODO: need to check number
    climberFollower(climberMotor1.GetDeviceID(), false)
{

    climberMotor2.SetControl(climberFollower);
    // Lift();
    // m_ClimberState = ClimberState::DOWN;

}

// This method will be called once per scheduler run
void Climber::Periodic() {
  frc::SmartDashboard::PutBoolean("Climber at Bot?",botLimit1.Get());
  frc::SmartDashboard::PutNumber("Climber_Position",climberMotor1.GetPosition().GetValueAsDouble());
/* if (!botLimit1.Get()) {
      climberMotor1.Set(0.0);
    }*/
}

void Climber::engage() {
  lockServo.SetAngle(30);
}

void Climber::disengage() {
  lockServo.SetAngle(45);
  time_brake_released = frc::GetTime();
}

void Climber::setAngle(double angle) {
  lockServo.SetAngle(angle);
}

void Climber::disableBrake() {
  
  lockServo.SetOffline();
  lockServo.SetDisabled(); 
  lockServo.SetPulseTime(0_ms);
  
}

void Climber::moveMotor() {
    climberMotor1.Set(0.6); // retract when set to 0.1
    // climberMotor2.Set(0.1); // retract when set to 0.1
}

void Climber::stopMotor() {
    climberMotor1.Set(0.0);
    // climberMotor2.Set(0.0);
}

void Climber::extend() {
  switch (m_ClimberState)
  {
    case CLIMBER_EXTEND_START:
    { 
      m_ClimberState = CLIMBER_EXTEND_BRAKE_DISENGAGE;
      disengage();
      break;
    }
    case CLIMBER_EXTEND_BRAKE_DISENGAGE:
    {
      if ((frc::GetTime() - time_brake_released).value() > 0.2)
      {
        m_ClimberState = CLIMBER_EXTEND_MOVING;
      }
      break;
    }
    case CLIMBER_EXTEND_MOVING: 
    {
      climberMotor1.Set(-0.6);
      if (fabs(climberMotor1.GetPosition().GetValueAsDouble()) >= 315)
      {
        //climberMotor1.StopMotor();
        climberMotor1.Set(0);
        m_ClimberState = CLIMBER_EXTEND_DONE;
      }
    }
    case CLIMBER_EXTEND_DONE:
    {
      engage();
      break;
    }
    default:
      break;
  }
}

void Climber::retract(){
    
    if (botLimit1.Get()) {
      climberMotor1.Set(0.6);
    }
    else {
      frc::SmartDashboard::PutBoolean("Climber",true);
      //climberMotor1.StopMotor();
      climberMotor1.Set(0);
      climberMotor1.SetPosition(units::angle::turn_t{0}); //TODO: Speed up later/set to position
    }
}


// void Climber::move_Climber(ClimberState movment){

//  m_ClimberState == movment;

// switch (m_ClimberState)
//   {
//     case UP:
//     {
//       disengage();
//       if (fabs(climberMotor1.GetPosition().GetValueAsDouble()) >= 3000)
//       {
//         climberMotor1.StopMotor();
//         m_ClimberState = ClimberState::CLIMBER_DONE;
//       }
//       break;
//     }
//     case DOWN:
//     { 
//       engage();
//       Lift();
//       break;
//     }
//     case CLIMBER_DONE:
//     {
//       engage();
//       break;
//     }
//     default:
//       break;
//   }
// }




