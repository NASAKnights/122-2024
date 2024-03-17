// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/Command.h>
#include <frc2/command/CommandHelper.h>
#include <subsystems/Arm.h>

class ArmDown
    : public frc2::CommandHelper<frc2::Command, ArmDown> {
public:
  ArmDown(ArmSubsystem* _arm);

  void Initialize() override;

  void Execute() override;

  void End(bool interrupted) override;

  bool IsFinished() override;
private:
  ArmSubsystem* m_arm;
};
