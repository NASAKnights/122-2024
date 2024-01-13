// Copyright (c) FRC Team 122. All Rights Reserved.

#include "subsystems/SwerveDrive.hpp"

#include <frc/kinematics/SwerveDriveOdometry.h>
#include <frc/smartdashboard/SmartDashboard.h>

#include "Constants.hpp"

SwerveDrive::SwerveDrive()
    : modules{{SwerveModule(ElectricalConstants::kFrontLeftDriveMotorID,
                            ElectricalConstants::kFrontLeftTurnMotorID,
                            ElectricalConstants::kFrontLeftEncoderID,
                            DriveConstants::kFrontLeftOffset),
               SwerveModule(ElectricalConstants::kFrontRightDriveMotorID,
                            ElectricalConstants::kFrontRightTurnMotorID,
                            ElectricalConstants::kFrontRightEncoderID,
                            DriveConstants::kFrontRightOffset),
               SwerveModule(ElectricalConstants::kBackLeftDriveMotorID,
                            ElectricalConstants::kBackLeftTurnMotorID,
                            ElectricalConstants::kBackLeftEncoderID,
                            DriveConstants::kBackLeftOffset),
               SwerveModule(ElectricalConstants::kBackRightDriveMotorID,
                            ElectricalConstants::kBackRightTurnMotorID,
                            ElectricalConstants::kBackRightEncoderID,
                            DriveConstants::kBackRightOffset)}},
      odometry{DriveConstants::kSwerveKinematics,
               frc::Rotation2d(units::degree_t{m_pigeon.GetAngle()}),
               {modules[0].GetPosition(), modules[1].GetPosition(),
                modules[2].GetPosition(), modules[3].GetPosition()},
               frc::Pose2d()},
      pidX{0.9, 1e-4, 0}, pidY{0.9, 1e-4, 0}, pidRot{0.15, 0, 0},
      networkTableInst(nt::NetworkTableInstance::GetDefault()) {
  navx.Calibrate();
  speeds = frc::ChassisSpeeds();
  networkTableInst.StartServer();

  poseTable = networkTableInst.GetTable("poseXD");
  ntPoseSubscribe = poseTable->GetDoubleArrayTopic(ntName).Subscribe(
      {}, {.periodic = 0.01, .sendAll = true});

  ntPosePublisher = ntPoseTopic.Publish();
}

// This method will be called once per scheduler run
void SwerveDrive::Periodic() {
  // getCameraResults();
  // sensor fusion? EKF (eek kinda fun)
  // publishOdometry(odometry.GetPose());
  frc::SmartDashboard::PutNumber("Heading", GetHeading().Degrees().value());
  UpdateOdometry();
}

void SwerveDrive::Drive(frc::ChassisSpeeds speeds) {
  auto states = DriveConstants::kSwerveKinematics.ToSwerveModuleStates(speeds);

  DriveConstants::kSwerveKinematics.DesaturateWheelSpeeds(
      &states, speeds, units::meters_per_second_t{ModuleConstants::kMaxSpeed},
      DriveConstants::kMaxTranslationalVelocity,
      DriveConstants::kMaxRotationalVelocity);

  for (int i = 0; i < 4; i++) {
    modules[i].SetDesiredState(states[i]);
  }

  frc::SmartDashboard::PutNumber("drive/vx", speeds.vx.value());
  frc::SmartDashboard::PutNumber("drive/vy", speeds.vy.value());
  frc::SmartDashboard::PutNumber("drive/omega", speeds.omega.value());

  frc::SmartDashboard::PutNumber("drive/x meters", odometry.GetPose().X().value());
  frc::SmartDashboard::PutNumber("drive/y meters", odometry.GetPose().Y().value());
  frc::SmartDashboard::PutNumber("drive/rotation degrees", odometry.GetPose().Rotation().Degrees().value());
}

void SwerveDrive::SetFast() {}

void SwerveDrive::SetSlow() {}

frc::Rotation2d SwerveDrive::GetHeading() {
  return m_pigeon.GetRotation2d();
}

void SwerveDrive::ResetHeading() { m_pigeon.Reset(); }

void SwerveDrive::ResetDriveEncoders() {
  for (auto &module : modules) {
    module.ResetDriveEncoders();
  }
}

std::array<frc::SwerveModulePosition, 4> SwerveDrive::GetModulePositions() {
  return std::array<frc::SwerveModulePosition, 4>{
      {modules[0].GetPosition(), modules[1].GetPosition(),
       modules[2].GetPosition(), modules[3].GetPosition()}};
}

void SwerveDrive::ResetPose(frc::Pose2d position) {
  odometry.ResetPosition(GetHeading(), GetModulePositions(), position);
}

frc::Pose2d SwerveDrive::GetPose() { return odometry.GetPose(); }

void SwerveDrive::UpdateOdometry() {
  odometry.Update(GetHeading(), GetModulePositions());
}

void SwerveDrive::InitializePID() {
  pidX = frc::PIDController(0.9, 1e-4, 0);
  pidY = frc::PIDController(0.9, 1e-4, 0);
  pidRot = frc::PIDController(0.15, 0, 0);

  pidX.SetTolerance(0.025);
  pidY.SetTolerance(0.025);
  pidRot.SetTolerance(1);

  hasRun = false;
}

void SwerveDrive::SetReference(frc::Pose2d desiredPose) {
  if ((!pidX.AtSetpoint() && !pidY.AtSetpoint()) | !hasRun) {
    speeds = frc::ChassisSpeeds{
        units::meters_per_second_t{
            pidX.Calculate(GetPose().X().value(), desiredPose.X().value())},
        units::meters_per_second_t{
            pidY.Calculate(GetPose().Y().value(), desiredPose.Y().value())},
        units::radians_per_second_t{0}};
    Drive(speeds);
  }
}

//--------------------------------------------

std::optional<frc::Pose3d> SwerveDrive::getCameraResults() {
  auto result = ntPoseSubscribe.GetAtomic();
  auto time = result.time; // time stamp

  if (time != 0.0) {
    auto compressedResults = result.value;
    return frc::Pose3d(
        frc::Translation3d(units::meter_t{compressedResults.at(0)},
                           units::meter_t{compressedResults.at(1)},
                           units::meter_t{compressedResults.at(2)}),
        frc::Rotation3d(units::radian_t{compressedResults.at(3)},
                        units::radian_t{compressedResults.at(4)},
                        units::radian_t{compressedResults.at(5)}));
  } else {
    return std::nullopt;
  }
}

void SwerveDrive::PublishOdometry(frc::Pose2d odometryPose) {
  double poseDeconstruct[]{double{odometryPose.X()}, double{odometryPose.Y()}};
  int64_t time = nt::Now();
  ntPosePublisher.Set(poseDeconstruct, time);
}

void SwerveDrive::PrintNetworkTableValues() {
  // TODO: write print function :3
}
