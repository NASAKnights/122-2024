// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/Intake.h"

Intake::Intake() :
    m_intakeMotor(8)
{   
    m_intakeMotor.ConfigVoltageCompSaturation(12);
    m_intakeMotor.EnableVoltageCompensation(true);
}

void Intake::Periodic() {}

void Intake::runIntake() {
    //Makes the intake do the funny :3
    m_intakeMotor.Set(ControlMode::PercentOutput, -0.55);
}

void Intake::intakeIndex() {
    m_intakeMotor.Set(ControlMode::PercentOutput, -0.85);
}

void Intake::runIntakeReverse() {
    //runIntake but reverse, makes the intake do the funny but in reverse.
    m_intakeMotor.Set(ControlMode::PercentOutput, 0.85);
}

void Intake::stopIntake() {
    m_intakeMotor.Set(ControlMode::PercentOutput, 0.0);
}