// Copyright (c) FRC Team 122. All Rights Reserved.

#pragma once

#include <array>

#include <AHRS.h>
#include <frc/SPI.h>
#include <frc/controller/PIDController.h>
#include <frc/geometry/Pose2d.h>
#include <frc/geometry/Pose3d.h>
#include <frc/geometry/Rotation3d.h>
#include <frc/geometry/Translation2d.h>
#include <frc/geometry/Translation3d.h>
#include <frc/kinematics/ChassisSpeeds.h>
#include <frc/kinematics/SwerveDriveKinematics.h>
#include <frc/kinematics/SwerveDriveOdometry.h>
#include <frc/kinematics/SwerveModulePosition.h>
#include <frc/kinematics/SwerveModuleState.h>
#include <frc2/command/SubsystemBase.h>
#include <frc/geometry/Quaternion.h>
#include <networktables/DoubleArrayTopic.h>
#include <networktables/NetworkTableInstance.h>
#include <wpi/array.h>
#include <iostream>
#include <string>
#include <ctre/phoenix6/Pigeon2.hpp>

#include <frc/estimator/SwerveDrivePoseEstimator.h>
#include <frc/estimator/PoseEstimator.h>
#include <frc/trajectory/constraint/SwerveDriveKinematicsConstraint.h>

#include "Constants.hpp"
#include "SwerveModule.hpp"

class SwerveDrive : public frc2::SubsystemBase {
public:
  SwerveDrive();

  /**
   * Will be called periodically whenever the CommandScheduler runs.
   */
  void Periodic() override;

  void Drive(frc::ChassisSpeeds);

  void SetFast();
  void SetSlow();

  frc::Rotation2d GetHeading();
  void ResetHeading();
  void ResetDriveEncoders();

  std::array<frc::SwerveModulePosition, 4> GetModulePositions();

  void ResetPose(frc::Pose2d);
  frc::Pose2d GetPose();

  void UpdateOdometry();

  void InitializePID();
  void SetReference(frc::Pose2d);

  void UpdatePoseEstimate();
  void PublishOdometry(frc::Pose2d);
  void PrintNetworkTablseValues();
  void SetVision();
  frc::Pose2d GetVision();

private:
  // Components (e.g. motor controllers and sensors) should generally be
  // declared private and exposed only through public methods.
  AHRS navx{frc::SPI::Port::kMXP};

  ctre::phoenix6::hardware::Pigeon2 m_pigeon{2, "NKCANivore"};

  std::array<SwerveModule, 4> modules;
  frc::SwerveDriveKinematics<4> kSwerveKinematics;

  frc::ChassisSpeeds speeds;
  // frc::SwerveDriveOdometry<4> odometry;

  frc::PIDController pidX;
  frc::PIDController pidY;
  frc::PIDController pidRot;

  bool hasRun = false;
  // ----------------------

  nt::NetworkTableInstance networkTableInst;
  
  std::string_view baseLink1 = "base_link_1";
  std::string_view baseLink2 = "base_link_2";
  std::string_view baseLink = "base_link";
  std::shared_ptr<nt::NetworkTable> poseTable;
  
  nt::DoubleArraySubscriber baseLink1Subscribe;
  nt::DoubleArraySubscriber baseLink2Subscribe;
  frc::Quaternion rotation_q; //w, x, y, z
  frc::SwerveDrivePoseEstimator<4> m_poseEstimator;

  nt::DoubleArrayPublisher baseLinkPublisher;
  
};
