// Copyright (c) FRC Team 122. All Rights Reserved.

#pragma once

#include <filesystem>
#include <map>
#include <string>
#include <vector>

#include <frc/Filesystem.h>
#include <wpi/json.h>

#include "util/NKTrajectory.hpp"

class NKTrajectoryManager {
public:
  static const NKTrajectory &GetTrajectory(const std::string &name);

private:
  std::map<std::string, NKTrajectory> LoadTrajectories();

  static NKTrajectory LoadFile(const std::filesystem::path &trajPath);

  std::map<std::string, NKTrajectory> m_trajectories;

  NKTrajectoryManager();

  static NKTrajectoryManager s_instance;

  NKTrajectoryManager(const NKTrajectoryManager &other) = delete;
  NKTrajectoryManager &operator=(const NKTrajectoryManager &other) = delete;
};
