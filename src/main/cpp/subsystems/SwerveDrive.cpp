// Copyright (c) FRC Team 122. All Rights Reserved.

#include "subsystems/SwerveDrive.hpp"

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
      kSwerveKinematics{{DriveConstants::kFrontLeftPosition, 
                        DriveConstants::kFrontRightPosition,
                        DriveConstants::kBackLeftPosition, 
                        DriveConstants::kBackRightPosition}},
      odometry{kSwerveKinematics,
               frc::Rotation2d(units::degree_t{m_pigeon.GetAngle()}),
               {modules[0].GetPosition(), modules[1].GetPosition(),
                modules[2].GetPosition(), modules[3].GetPosition()},
               frc::Pose2d()},
      pidX{0.9, 1e-4, 0}, pidY{0.9, 1e-4, 0}, pidRot{0.15, 0, 0},
      networkTableInst(nt::NetworkTableInstance::GetDefault()),
      m_poseEstimator{
          kSwerveKinematics,
          frc::Rotation2d(units::degree_t{m_pigeon.GetAngle()}),
          {modules[0].GetPosition(), modules[1].GetPosition(),
            modules[2].GetPosition(), modules[3].GetPosition()},
          frc::Pose2d()} 
{
  navx.Calibrate();
  speeds = frc::ChassisSpeeds();
  networkTableInst.StartServer();

  poseTable = networkTableInst.GetTable("ROS2Bridge");
  ntPoseSubscribe = poseTable->GetDoubleArrayTopic(ntName).Subscribe(
      {}, {.periodic = 0.01, .sendAll = true});

  ntPosePublisher = ntPoseTopic.Publish();
}

// This method will be called once per scheduler run
void SwerveDrive::Periodic() {
  // getCameraResults();
  // sensor fusion? EKF (eek kinda fun) (extended Kalman filter)
  // publishOdometry(odometry.GetPose());

  PrintNetworkTableValues();

  frc::SmartDashboard::PutNumber("Heading", GetHeading().Degrees().value());
  UpdateOdometry();

}

void SwerveDrive::Drive(frc::ChassisSpeeds speeds) {
  auto states = kSwerveKinematics.ToSwerveModuleStates(speeds);

  kSwerveKinematics.DesaturateWheelSpeeds(
      &states, speeds, units::meters_per_second_t{ModuleConstants::kMaxSpeed},
      DriveConstants::kMaxTranslationalVelocity,
      DriveConstants::kMaxRotationalVelocity);

  for (int i = 0; i < 4; i++) {
    modules[i].SetDesiredState(states[i]);
  }

  frc::SmartDashboard::PutNumber("drive/vx", speeds.vx.value());
  frc::SmartDashboard::PutNumber("drive/vy", speeds.vy.value());
  frc::SmartDashboard::PutNumber("drive/omega", speeds.omega.value());

  // frc::SmartDashboard::PutNumber("drive/x meters", odometry.GetPose().X().value());
  // frc::SmartDashboard::PutNumber("drive/y meters", odometry.GetPose().Y().value());
  // frc::SmartDashboard::PutNumber("drive/rotation degrees", odometry.GetPose().Rotation().Degrees().value());
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

void SwerveDrive::UpdatePoseEstimate() {
  auto result = ntPoseSubscribe.GetAtomic();
  // auto time = result.time; // time stamp

  if (result.value.size() > 0) {
    auto compressedResults = result.value;
    rotation_q = frc::Quaternion(compressedResults.at(6),
                                     compressedResults.at(3),
                                     compressedResults.at(4),
                                     compressedResults.at(5));

    auto posTranslation =frc::Translation3d(units::meter_t{compressedResults.at(0)},
                           units::meter_t{compressedResults.at(1)},
                           units::meter_t{compressedResults.at(2)});
    frc::Pose3d cameraPose = frc::Pose3d(posTranslation, frc::Rotation3d(rotation_q));
    frc::Pose2d visionMeasurement2d = cameraPose.ToPose2d();
    m_poseEstimator.AddVisionMeasurement(visionMeasurement2d, 
                          units::second_t {compressedResults.at(7)});
  }

  m_poseEstimator.Update(m_pigeon.GetRotation2d(),GetModulePositions());
}

void SwerveDrive::PublishOdometry(frc::Pose2d odometryPose) {
  double poseDeconstruct[]{double{odometryPose.X()}, double{odometryPose.Y()}};
  int64_t time = nt::Now();
  ntPosePublisher.Set(poseDeconstruct, time);
}

void SwerveDrive::PrintNetworkTableValues() {
  // TODO: write print function :3
  
}
