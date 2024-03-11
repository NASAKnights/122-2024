// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/Command.h>
#include <frc2/command/CommandHelper.h>
#include <subsystems/Shooter.h>
#include <subsystems/Indexer.h>
#include <subsystems/Intake.h>
#include <subsystems/SwerveDrive.hpp>
#include <arm/Arm.h>

enum SmartShooterState
{
  SMARTSPINUP,
  SMARTSHOOTING,
  SMARTDONE
};

class SmartShoot
    : public frc2::CommandHelper<frc2::Command, SmartShoot> {
 public:
  SmartShoot(Shooter* _shooter, Indexer* _indexer, Intake* _intake, ArmSubsystem* _arm, double _shootSpeed,  SwerveDrive* _swerdrive);

  void Initialize() override;

  void Execute() override;

  void End(bool interrupted) override;

  bool IsFinished() override;
  
  void Arm_Position();
  private:
    Shooter* shoooter;
    Indexer* indexing;
    Intake* intake;
    ArmSubsystem* arm;
    SwerveDrive* swerdrive;

    double shootSpeed;
    float distance;
    SmartShooterState m_state;
};
