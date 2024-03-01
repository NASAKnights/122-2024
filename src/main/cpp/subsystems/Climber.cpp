// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.ßöä
#include "subsystems/Climber.h"

Climber::Climber() : 
    climberMotor1(3),
    climberMotor2(4),
    lockServo(3), // TODO: need to check number
    climberFollower(climberMotor1.GetDeviceID(), false)
{

    climberMotor2.SetControl(climberFollower);
    Zero();
    m_ClimberState = ClimberState::DOWN;

}

// This method will be called once per scheduler run
void Climber::Periodic() {}

void Climber::Zero(){
    while(!bottom.Get()){
    climberMotor1.Set(-1.0);
    }
    climberMotor1.StopMotor();
    climberMotor1.SetPosition(units::angle::turn_t{0});
}

void Climber::down() {

}

void Climber::lock() {
lockServo.SetAngle(145);
}

void Climber::unlock() {
lockServo.SetAngle(100);
}

void Climber::moveMotor() {
    climberMotor1.Set(0.1); // retract when set to 0.1
    // climberMotor2.Set(0.1); // retract when set to 0.1
}

void Climber::stopMotor() {
    climberMotor1.Set(0.0);
    // climberMotor2.Set(0.0);
}
void Climber::move_Climber(ClimberState movment){

  m_ClimberState == movment;

switch (m_ClimberState)
  {
  case UP:
  {
   unlock();
  if (climberMotor1.GetPosition().GetValueAsDouble() >= 5000)
  {
    climberMotor1.StopMotor();
    m_ClimberState = ClimberState::CLIMBER_DONE;
  }
  break;
  }
  case DOWN:
  { lock();
    Zero();
  break;
  }
  case CLIMBER_DONE:
  {
   lock();
  break;
  }
  default:
    break;
}
}




