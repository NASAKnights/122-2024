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
#include "subsystems/SwerveDrive.hpp"

enum ShooterState
{
  SPINUP,
  SHOOTING,
  DONE
};

// class Solver: public frc2::CommandHelper<frc2::Command
//   public:

class NewShoot
    : public frc2::CommandHelper<frc2::Command, NewShoot> {
 public:
  NewShoot(Shooter*, Indexer*, Intake*, ArmSubsystem*);

  void Initialize() override;

  void Execute() override;

  void End(bool interrupted) override;

  bool IsFinished() override;

  private:
    Shooter* shoooter;
    Indexer* indexing;
    Intake* intake;
    ArmSubsystem* arm;
    frc::Pose2d targetPose;

    bool ValidateState(ArmSubsystem*, Shooter*, SwerveDrive*);
    
    ShooterState m_state;
};
