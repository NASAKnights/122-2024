// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/Command.h>
#include <frc2/command/CommandHelper.h>
#include <subsystems/Shooter.h>
#include <subsystems/Indexer.h>
#include <subsystems/Intake.h>
#include <subsystems/Arm.h>

enum TrapShooterState
{
  TrapSPINUP,
  TrapSHOOTING,
  TrapDONE
};

class Trapshoot
    : public frc2::CommandHelper<frc2::Command, Trapshoot> {
 public:
  Trapshoot(Shooter* _shooter, Indexer* _indexer, 
      Intake* _intake, ArmSubsystem* _arm,
      double _shootSpeed, double _shootAngle, units::second_t _spinupTime,double diff);

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
    double diff;

    units::second_t shooterSpinupTime;

    frc::Timer spinupTime;
    
   TrapShooterState m_state;
};
