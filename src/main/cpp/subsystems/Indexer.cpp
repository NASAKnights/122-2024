// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/Indexer.h"

Indexer::Indexer() : limitSwitchNear(2), limitSwitchFar(3)
{
    wpi::log::DataLog &log = frc::DataLogManager::GetLog();
    m_NoteLog = wpi::log::BooleanLogEntry(log, "/Indexer/Note");
}

// This method will be called once per scheduler run
void Indexer::Periodic()
{
    frc::SmartDashboard::PutBoolean("note", hasNote());
    m_NoteLog.Append(hasNote());
}

bool Indexer::hasNote()
{
    return (limitSwitchNear.Get() || limitSwitchFar.Get());
}
/*
bool Indexer::FarNote() {
       if

}*/