// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/Command.h>
#include <frc2/command/CommandHelper.h>
#include <subsystems/Shooter.h>
#include <subsystems/Indexer.h>
#include <subsystems/Intake.h>
#include <subsystems/LEDController.h>
#include <subsystems/Arm.h>

class MotorRun
    : public frc2::CommandHelper<frc2::Command, MotorRun> {
 public:
  MotorRun(Shooter* _shooter, double _shootSpeed, units::second_t _spinupTime);

  void Initialize() override;

  void Execute() override;

  void End(bool interrupted) override;

  bool IsFinished() override;

  private:
    Shooter* shoooter;
    Indexer* indexing;
    Intake* intake;
    ArmSubsystem* arm;
    LEDController* m_led_control;

    double shootSpeed;
    double shootAngle;

    units::second_t shooterSpinupTime;

    frc::Timer spinupTime;
};
