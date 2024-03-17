// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/Command.h>
#include <frc2/command/CommandHelper.h>

#include "subsystems/Indexer.h"
#include "subsystems/Intake.h"
#include "subsystems/Shooter.h"
#include "subsystems/Arm.h"
#include "subsystems/LEDController.h"

enum IntakeState
{
  MOVING,
  IDLE
};

class IntakeNote
    : public frc2::CommandHelper<frc2::Command, IntakeNote> {
 public:
  IntakeNote(Intake* _intake, Indexer* _indexer, ArmSubsystem* _arm, LEDController* led_controller);

  void Initialize() override;

  void Execute() override;

  void End(bool interrupted) override;

  bool IsFinished() override;

private:
  Intake* intake;
  Shooter* shooter;
  Indexer* indexer;
  ArmSubsystem* m_arm;
  LEDController* m_led_control;

  IntakeState m_state;
};
