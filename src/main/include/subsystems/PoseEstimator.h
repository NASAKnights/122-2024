// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/SubsystemBase.h>
#include <frc/geometry/Translation2d.h>
#include <frc/geometry/Translation3d.h>
#include <networktables/DoubleArrayTopic.h>
#include <networktables/NetworkTableInstance.h>
#include <frc/geometry/Pose2d.h>
#include <frc/geometry/Pose3d.h>
#include <frc/geometry/Rotation3d.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/smartdashboard/Field2d.h>

class PoseEstimator : public frc2::SubsystemBase {
 public:
  PoseEstimator();
  void UpdateMeasurement();
  frc::Pose2d GetMeasurement(std::string key);


  /**
   * Will be called periodically whenever the CommandScheduler runs.
   */
  void Periodic() override;

 private:
  // Components (e.g. motor controllers and sensors) should generally be
  // declared private and exposed only through public methods.
  nt::NetworkTableInstance networkTableInst;
  std::string_view objectLink = "note";
  std::string_view robotPoseLink = "base_link";
  nt::DoubleArraySubscriber robot2ObjectSubscribe;
  nt::DoubleArraySubscriber robotPoseSubscribe;
  std::vector<double> prevValue = {};

};
