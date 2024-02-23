// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/Indexer.h"

Indexer::Indexer() : limitSwitch(2) {

}

// This method will be called once per scheduler run
void Indexer::Periodic() {}

bool Indexer::hasNote() {
    return limitSwitch.Get();
}