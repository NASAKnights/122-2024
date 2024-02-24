// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/Command.h>
#include <frc2/command/CommandHelper.h>
#include <subsystems/Intake.h>
#include <subsystems/Shooter.h>
#include <subsystems/Indexer.h>
#include <arm/Arm.h>

/**
 * An example command.
 *
 * <p>Note that this extends CommandHelper, rather extending Command
 * directly; this is crucially important, or else the decorator functions in
 * Command will *not* work!
 */
enum IntakeState
{
  MOVING,
  IDLE
};

class intakeTake
    : public frc2::CommandHelper<frc2::Command, intakeTake> {
 public:
  intakeTake(Intake* _intake, Indexer* _indexer, ArmSubsystem* _arm);

  void Initialize() override;

  void Execute() override;

  void End(bool interrupted) override;

  bool IsFinished() override;

private:
  Intake* intake;
  Shooter* shooter;
  Indexer* indexer;
  ArmSubsystem* m_arm;

  IntakeState m_state;
};
