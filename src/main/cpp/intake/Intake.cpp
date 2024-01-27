// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "intake/Intake.h"

Intake::Intake() {
    // m_intakeMotor.SetNeutralMode(ctre::phoenix6::signals::NeutralModeValue::Brake)
}

void Intake::Periodic() {/*IDK what I should put here *yet*/}

void Intake::runIntake() {
    //Makes the intake do the funny :3
    m_intakeMotor.Set(0.1);
}

void Intake::runIntakeReverse() {
    //runIntake but reverse, makes the intake do the funny but in reverse.
    m_intakeMotor.Set(-0.1);
}

void Intake::stopIntake() {
    m_intakeMotor.Set(0);
}