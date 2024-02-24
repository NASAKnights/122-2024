// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/Command.h>
#include <frc2/command/CommandHelper.h>
#include <subsystems/Shooter.h>
#include <subsystems/Indexer.h>
#include <subsystems/Intake.h>
#include <arm/Arm.h>

enum ShooterState
{
  SPINUP,
  SHOOTING,
  DONE
};

class Shoot
    : public frc2::CommandHelper<frc2::Command, Shoot> {
 public:
  Shoot(Shooter* _shooter, Indexer* _indexer, Intake* _intake, ArmSubsystem* _arm, double _shootSpeed, double _shootAngle);

  void Initialize() override;

  void Execute() override;

  void End(bool interrupted) override;

  bool IsFinished() override;

  private:
    Shooter* shoooter;
    Indexer* indexing;
    Intake* intake;
    ArmSubsystem* arm;

    double shootSpeed;
    double shootAngle;
    
    ShooterState m_state;
};
