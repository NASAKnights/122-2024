// Copyright (c) FRC Team 122. All Rights Reserved.

#pragma once

#include <frc/Timer.h>
#include <frc/controller/PIDController.h>
#include <frc2/command/Command.h>
#include <frc2/command/CommandHelper.h>

#include "Subsystems/SwerveDrive.hpp"
#include "util/NKTrajectory.hpp"

class TrajectoryFollower
    : public frc2::CommandHelper<frc2::Command, TrajectoryFollower> {
public:
  TrajectoryFollower(SwerveDrive *drive, const NKTrajectory *trajectory);

  void Initialize() override;

  void Execute() override;

  void End(bool interrupted) override;

  bool IsFinished() override;

private:
  SwerveDrive *m_drive;

  const NKTrajectory *m_trajectory;

  frc::Timer m_timestamp;

  frc::PIDController m_controllerX{6.0, 0, 0};
  frc::PIDController m_controllerY{6.0, 0, 0};
  frc::PIDController m_controllerRotation{10.0, 0, 0};
};
