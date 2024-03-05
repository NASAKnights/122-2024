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
      // odometry{kSwerveKinematics,
      //          frc::Rotation2d(units::degree_t{m_pigeon.GetAngle()}),
      //          {modules[0].GetPosition(), modules[1].GetPosition(),
      //           modules[2].GetPosition(), modules[3].GetPosition()},
      //          frc::Pose2d()},
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
  baseLink1Subscribe = poseTable->GetDoubleArrayTopic(baseLink1).Subscribe(
      {}, {.periodic = 0.01, .sendAll = true});
  baseLink2Subscribe = poseTable->GetDoubleArrayTopic(baseLink2).Subscribe(
      {}, {.periodic = 0.01, .sendAll = true});

  baseLinkPublisher = poseTable->GetDoubleArrayTopic(baseLink).Publish();
}

// This method will be called once per scheduler run
void SwerveDrive::Periodic() {
  // getCameraResults();
  // sensor fusion? EKF (eek kinda fun) (extended Kalman filter)
  
  PublishOdometry(m_poseEstimator.GetEstimatedPosition());
  UpdatePoseEstimate();
  
  //PrintNetworkTableValues();

  frc::SmartDashboard::PutNumber("Heading", GetHeading().Degrees().value());
  // UpdateOdometry();

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
  // odometry.ResetPosition(GetHeading(), GetModulePositions(), position);
  m_poseEstimator.ResetPosition(GetHeading(), GetModulePositions(), position);
}

frc::Pose2d SwerveDrive::GetPose() { return m_poseEstimator.GetEstimatedPosition(); }

void SwerveDrive::UpdateOdometry() {
  // odometry.Update(GetHeading(), GetModulePositions());
}
void SwerveDrive::SetVision(){ 
   
  m_poseEstimator.ResetPosition(m_pigeon.GetRotation2d(), GetModulePositions(), GetVision());

  
}

frc::Pose2d SwerveDrive::GetVision() {
  auto result2 = baseLink2Subscribe.GetAtomic();
  // auto time = result.time; // time stamp
  if (result2.value.size() > 0) {
    auto compressedResults = result2.value;
    rotation_q = frc::Quaternion(compressedResults.at(6),
                                     compressedResults.at(3),
                                     compressedResults.at(4),
                                     compressedResults.at(5));

    auto posTranslation =frc::Translation3d(units::meter_t{compressedResults.at(0)},
                           units::meter_t{compressedResults.at(1)},
                           units::meter_t{compressedResults.at(2)});
    frc::Pose3d cameraPose = frc::Pose3d(posTranslation, frc::Rotation3d(rotation_q));
    frc::Pose2d visionMeasurement2d = cameraPose.ToPose2d();
    return  visionMeasurement2d;
  }
  else{

    return frc::Pose2d{};
  }
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
  auto result1 = baseLink1Subscribe.GetAtomic();
  auto result2 = baseLink2Subscribe.GetAtomic();
  // auto time = result.time; // time stamp

  if (result1.value.size() > 0) {
    auto compressedResults = result1.value;
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
  if (result2.value.size() > 0) {
    auto compressedResults = result2.value;
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
  double time = nt::Now()/(1e6);
  Eigen::Vector3d odoRotation = Eigen::Vector3d(0.0, 0.0, double(odometryPose.Rotation().Radians()));
  frc::Quaternion odoPoseQ = frc::Quaternion::FromRotationVector(odoRotation);
  double poseDeconstruct[]{double{odometryPose.X()}, double{odometryPose.Y()},0.0,
        odoPoseQ.X(),odoPoseQ.Y(),odoPoseQ.Z(),odoPoseQ.W(), time};
  baseLinkPublisher.Set(poseDeconstruct, time);
}
/*
void SwerveDrive::PrintNetworkTableValues() {
  // TODO: write print function :3
  
}
*/