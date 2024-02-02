// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/Shooter.h"

bool running = false;

Shooter::Shooter() { /*yes*/ }

void Shooter::Periodic() {}

void Shooter::shoot() {
    m_shooterMotor.Set(0.1);
}

void Shooter::stopShooter() {
    m_shooterMotor.Set(0);
}

double Shooter::getSpeed() {
    return double {m_shooterMotor.GetVelocity().GetValue()};
}

bool Shooter::isRunning() {
    return running;
}