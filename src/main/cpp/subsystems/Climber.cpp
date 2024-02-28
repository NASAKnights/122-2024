// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/Climber.h"

Climber::Climber() : 
    climberMotor1(3),
    climberMotor2(4),
    lockServo(3) // TODO: need to check number
{

}

// This method will be called once per scheduler run
void Climber::Periodic() {}

void Climber::down() {

}

void Climber::lock() {

}

void Climber::unlock() {

}


