// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.ßöä
#include "subsystems/Climber.h"

Climber::Climber() : 
    climberMotor1(3),
    climberMotor2(4),
    lockServo(9),
    climberFollower(climberMotor1.GetDeviceID(), false)
{

    climberMotor2.SetControl(climberFollower);

}

// This method will be called once per scheduler run
void Climber::Periodic() {
  frc::SmartDashboard::PutBoolean("Climber at Bot?",botLimit1.Get());
  frc::SmartDashboard::PutNumber("Climber_Position",climberMotor1.GetPosition().GetValueAsDouble());

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
    climberMotor1.Set(0.1); // retracts when set to 0.1
}

void Climber::stopMotor() {
    climberMotor1.Set(0.0);
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
      climberMotor1.Set(-0.9);
      if (fabs(climberMotor1.GetPosition().GetValueAsDouble()) >= 300)
      {
        climberMotor1.Set(0);
        m_ClimberState = CLIMBER_EXTEND_DONE;
      }
      break;
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
  switch (m_ClimberState)
  {
    case CLIMBER_RETRACT_START:
    { 
      m_ClimberState = CLIMBER_RETRACT_MOVING;
  
      break;
    }
    case CLIMBER_RETRACT_MOVING: 
    {
      climberMotor1.Set(0.9);
      // Soft Limit
      if (fabs(climberMotor1.GetPosition().GetValueAsDouble()) <= 75)
      {
        climberMotor1.Set(0);
        m_ClimberState = CLIMBER_RETRACT_DONE;
      }
      break;
    }
    case CLIMBER_RETRACT_DONE:
    {
      break;
    }
    default:
      break;
  
}
}

void Climber::retractLimit_Pit(){
    
    if (botLimit1.Get()) {
      climberMotor1.Set(0.1);
    }
    else {
      //climberMotor1.StopMotor();
      climberMotor1.Set(0);
      while(climberMotor1.SetPosition(units::angle::turn_t{0}) != ctre::phoenix::StatusCode::OK){};
    }
}

bool Climber::atBot() {
  return (!botLimit1.Get());
}





