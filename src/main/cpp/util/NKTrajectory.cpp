// Copyright (c) FRC Team 122. All Rights Reserved.

#include "util/NKTrajectory.hpp"

#include <algorithm>
#include <vector>

#include <frc/geometry/Pose2d.h>
#include <frc/geometry/Twist2d.h>
#include <units/angle.h>
#include <units/angular_velocity.h>
#include <units/base.h>
#include <units/time.h>
#include <units/velocity.h>
#include <wpi/json.h>

using namespace units;
using namespace frc;
using wpi::json;

NKTrajectory::State
NKTrajectory::State::Interpolate(const NKTrajectory::State &other,
                                 second_t newT) const {
  double scale = ((newT - t) / (other.t - t)).value();
  return NKTrajectory::State{newT, pose.Exp(pose.Log(other.pose) * scale),
                             (other.vx - vx) * scale + vx,
                             (other.vy - vy) * scale + vy,
                             (other.omega - omega) * scale + omega};
  return other;
}

NKTrajectory::NKTrajectory(std::vector<NKTrajectory::State> states)
    : m_states{states} {}

NKTrajectory::State NKTrajectory::Sample(second_t t) const {
  if (t < m_states.at(0).t) {
    return m_states.at(0);
  }
  if (t > GetTotalTime()) {
    return m_states.back();
  }

  auto upper = std::upper_bound(
      m_states.begin(), m_states.end(), t,
      [](second_t a, const NKTrajectory::State &b) { return a < b.t; });

  auto previousState = *(upper - 1);
  auto currentState = *upper;

  if ((currentState.t - previousState.t).value() == 0) {
    return currentState;
  }

  return previousState.Interpolate(currentState, t);
}

Pose2d NKTrajectory::GetInitialPose() const { return m_states.at(0).pose; }

second_t NKTrajectory::GetTotalTime() const { return m_states.back().t; }

void from_json(const json &j, NKTrajectory::State &state) {
  state.t = second_t{j.at("timestamp").get<double>()};
  state.pose = Pose2d(Translation2d(meter_t{j.at("x").get<double>()},
                                    meter_t{j.at("y").get<double>()}),
                      Rotation2d(radian_t{j.at("heading").get<double>()}));
  state.vx = meters_per_second_t{j.at("velocityX").get<double>()};
  state.vy = meters_per_second_t{j.at("velocityY").get<double>()};
  state.omega = radians_per_second_t{j.at("angularVelocity").get<double>()};
}

void to_json(json &j, const NKTrajectory::State &state) {
  j = json{{"timestamp", state.t.value()},
           {"x", state.pose.X().value()},
           {"y", state.pose.Y().value()},
           {"heading", state.pose.Rotation().Radians().value()},
           {"velocityX", state.vx.value()},
           {"velocityY", state.vy.value()},
           {"angularVelocity", state.omega.value()}};
}

void from_json(const json &j, NKTrajectory &traj) {
  traj = NKTrajectory(j.at("samples").get<std::vector<NKTrajectory::State>>());
}

void to_json(json &j, const NKTrajectory &traj) { j = json{traj.m_states}; }
