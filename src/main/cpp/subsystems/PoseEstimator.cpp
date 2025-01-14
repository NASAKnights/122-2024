// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/PoseEstimator.h"

PoseEstimator::PoseEstimator():networkTableInst(nt::NetworkTableInstance::GetDefault())
{
    auto poseTable = networkTableInst.GetTable("ROS2Bridge");

    robot2ObjectSubscribe = poseTable->GetDoubleArrayTopic(objectLink).Subscribe(
      {}, {.periodic = 0.01, .sendAll = true});

    robotPoseSubscribe = poseTable->GetDoubleArrayTopic(robotPoseLink).Subscribe(
      {}, {.periodic = 0.01, .sendAll = true});
}

// This method will be called once per scheduler run
void PoseEstimator::Periodic() {
  auto value = robot2ObjectSubscribe.Get();

    if (value != prevValue) {
      prevValue = value;  // save previous value
      PoseEstimator::UpdateMeasurement();
    }
}

void PoseEstimator::UpdateMeasurement()
{
  auto objectTranslationValue = robot2ObjectSubscribe.GetAtomic();
  auto robotPoseValue = robotPoseSubscribe.GetAtomic();

  if(objectTranslationValue.value.size() > 0) {
    auto objectX = units::length::meter_t(objectTranslationValue.value.at(0));
    auto objectY = units::length::meter_t(objectTranslationValue.value.at(1));

    frc::Rotation2d rotationToNote = frc::Rotation2d(units::math::atan2(objectX, objectY));

    units::length::meter_t robotPoseX = units::length::meter_t(robotPoseValue.value.at(0));
    units::length::meter_t robotPoseY = units::length::meter_t(robotPoseValue.value.at(1));
    frc::Rotation2d robotPoseO = units::angle::radian_t(robotPoseValue.value.at(2));

    frc::Pose2d robotPose = frc::Pose2d(robotPoseX, robotPoseY, robotPoseO);
    frc::Transform2d object = frc::Transform2d(objectX, objectY, rotationToNote);

    frc::Pose2d objectPose = robotPose.TransformBy(object);

  }
}
