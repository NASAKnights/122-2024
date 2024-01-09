// Copyright (c) FRC Team 122. All Rights Reserved.

#include "commands/TrajectoryFollower.hpp"

#include <frc/Timer.h>
#include <frc/kinematics/ChassisSpeeds.h>
#include <units/angular_velocity.h>
#include <units/velocity.h>

using namespace units;

TrajectoryFollower::TrajectoryFollower(SwerveDrive *drive,
                                       const NKTrajectory *trajectory)
    : m_drive{drive}, m_trajectory{trajectory} {
  AddRequirements(m_drive);
}

void TrajectoryFollower::Initialize() {
  m_timestamp.Reset();
  m_timestamp.Start();

  m_controllerRotation.EnableContinuousInput(-std::numbers::pi,
                                             std::numbers::pi);

  m_drive->ResetPose(m_trajectory->GetInitialPose());
}

void TrajectoryFollower::Execute() {
  auto state = m_trajectory->Sample(m_timestamp.Get());
  auto currentPose = m_drive->GetPose();

  m_controllerX.SetSetpoint(state.pose.X().value());
  m_controllerY.SetSetpoint(state.pose.Y().value());
  m_controllerRotation.SetSetpoint(state.pose.Rotation().Radians().value());

  auto vx = state.vx + meters_per_second_t{
                           m_controllerX.Calculate(currentPose.X().value())};
  auto vy = state.vy + meters_per_second_t{
                           m_controllerY.Calculate(currentPose.Y().value())};
  auto omega =
      state.omega + radians_per_second_t{m_controllerRotation.Calculate(
                        currentPose.Rotation().Radians().value())};

  m_drive->Drive(frc::ChassisSpeeds::FromFieldRelativeSpeeds(
      vx, vy, omega, currentPose.Rotation()));
}

void TrajectoryFollower::End(bool interrupted) {
  m_drive->Drive(frc::ChassisSpeeds{});
}

bool TrajectoryFollower::IsFinished() {
  return m_timestamp.Get() > m_trajectory->GetTotalTime();
}
