// Copyright (c) FRC Team 122. All Rights Reserved.

#pragma once

#include <array>

#include <AHRS.h>
#include <ctre/phoenix6/Pigeon2.hpp>
#include <frc/SPI.h>
#include <frc/controller/PIDController.h>
#include <frc/geometry/Pose2d.h>
#include <frc/geometry/Pose3d.h>
#include <frc/geometry/Quaternion.h>
#include <frc/geometry/Rotation3d.h>
#include <frc/geometry/Translation2d.h>
#include <frc/geometry/Translation3d.h>
#include <frc/kinematics/ChassisSpeeds.h>
#include <frc/kinematics/SwerveDriveKinematics.h>
#include <frc/kinematics/SwerveDriveOdometry.h>
#include <frc/kinematics/SwerveModulePosition.h>
#include <frc/kinematics/SwerveModuleState.h>
#include <frc/smartdashboard/Field2d.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc2/command/RunCommand.h>
#include <frc2/command/SubsystemBase.h>
#include <iostream>
#include <networktables/DoubleArrayTopic.h>
#include <networktables/NetworkTableInstance.h>
#include <string>
#include <wpi/array.h>

#include <frc/DriverStation.h>
#include <frc/estimator/PoseEstimator.h>
#include <frc/estimator/SwerveDrivePoseEstimator.h>
#include <frc/trajectory/constraint/SwerveDriveKinematicsConstraint.h>

#include <pathplanner/lib/auto/AutoBuilder.h>
#include <pathplanner/lib/util/HolonomicPathFollowerConfig.h>
#include <pathplanner/lib/util/PIDConstants.h>
#include <pathplanner/lib/util/ReplanningConfig.h>

#include "Constants.hpp"
#include "SwerveModule.hpp"

class SwerveDrive : public frc2::SubsystemBase
{
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
    void EnableDrive();
    void DisableDrive();

    std::array<frc::SwerveModulePosition, 4> GetModulePositions();

    void ResetPose(frc::Pose2d position);

    frc::Pose2d GetPose();

    void UpdateOdometry();
    frc::ChassisSpeeds getRobotRelativeSpeeds();

    void InitializePID();
    void SetReference(frc::Pose2d);
    void Strafe(frc::ChassisSpeeds speeds, double angle);

    void UpdatePoseEstimate();
    void PublishOdometry(frc::Pose2d);
    void PrintNetworkTablseValues();
    void SetVision();
    bool atSetpoint();
    frc::Pose2d GetVision();
    void TurnVisionOn();
    void TurnVisionOff();

  private:
    // Components (e.g. motor controllers and sensors) should generally be
    // declared private and exposed only through public methods.
    AHRS navx{frc::SPI::Port::kMXP};

    ctre::phoenix6::hardware::Pigeon2 m_pigeon{2, "NKCANivore"};

    std::array<SwerveModule, 4> modules;
    frc::SwerveDriveKinematics<4> kSwerveKinematics;

    frc::ChassisSpeeds speeds;
    frc::Field2d m_field;
    frc::PIDController pidX;
    frc::PIDController pidY;
    frc::PIDController pidRot;

    bool hasRun = false;
    bool enable = true;
    double pos_Error;

    bool useVision = false;

    frc::ChassisSpeeds priorSpeeds = frc::ChassisSpeeds();

    nt::NetworkTableInstance networkTableInst;

    std::string_view baseLink1 = "base_link_1";
    std::string_view baseLink2 = "base_link_2";
    std::string_view baseLink = "base_link";
    std::shared_ptr<nt::NetworkTable> poseTable;

    nt::DoubleArraySubscriber baseLink1Subscribe;
    nt::DoubleArraySubscriber baseLink2Subscribe;
    frc::Quaternion rotation_q; // w, x, y, z
    frc::SwerveDrivePoseEstimator<4> m_poseEstimator;

    nt::DoubleArrayPublisher baseLinkPublisher;
};
