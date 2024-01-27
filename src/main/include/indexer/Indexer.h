// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/SubsystemBase.h>
#include <frc/DigitalInput.h>
#include <ctre/phoenix6/TalonFX.hpp>

class Indexer : public frc2::SubsystemBase {
 public:
  Indexer();

  /**
   * Will be called periodically whenever the CommandScheduler runs.
   */
  void Periodic() override;

  bool getIndex();
  void moveIndexer();

 private:
  // Components (e.g. motor controllers and sensors) should generally be
  // declared private and exposed only through public methods.
  ctre::phoenix6::hardware::TalonFX m_indexerMotor{11};
  frc::DigitalInput limitSwitch{0};
};
