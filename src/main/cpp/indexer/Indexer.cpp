// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "indexer/Indexer.h"

Indexer::Indexer() = default;

// This method will be called once per scheduler run
void Indexer::Periodic() {}

bool Indexer::getIndex() {
    return limitSwitch.Get();
}

void Indexer::moveIndexer() {
    m_indexerMotor.Set(0.1);
}