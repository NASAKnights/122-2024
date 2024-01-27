// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "shooter/Shooter.h"

Shooter::Shooter() {
    //yes
}

void Shooter::Periodic() {}

void Shooter::shoot() {
    m_shooterMotor.Set(0.1);
}

void Shooter::stopShooter() {
    m_shooterMotor.Set(0);
}