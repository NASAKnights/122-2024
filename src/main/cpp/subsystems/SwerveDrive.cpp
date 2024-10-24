// Copyright (c) FRC Team 122. All Rights Reserved.

#include "subsystems/SwerveDrive.hpp"

SwerveDrive::SwerveDrive()
    : modules{{SwerveModule(ElectricalConstants::kFrontLeftDriveMotorID, ElectricalConstants::kFrontLeftTurnMotorID,
                            ElectricalConstants::kFrontLeftEncoderID, DriveConstants::kFrontLeftOffset),
               SwerveModule(ElectricalConstants::kFrontRightDriveMotorID, ElectricalConstants::kFrontRightTurnMotorID,
                            ElectricalConstants::kFrontRightEncoderID, DriveConstants::kFrontRightOffset),
               SwerveModule(ElectricalConstants::kBackLeftDriveMotorID, ElectricalConstants::kBackLeftTurnMotorID,
                            ElectricalConstants::kBackLeftEncoderID, DriveConstants::kBackLeftOffset),
               SwerveModule(ElectricalConstants::kBackRightDriveMotorID, ElectricalConstants::kBackRightTurnMotorID,
                            ElectricalConstants::kBackRightEncoderID, DriveConstants::kBackRightOffset)}},
      kSwerveKinematics{{DriveConstants::kFrontLeftPosition, DriveConstants::kFrontRightPosition,
                         DriveConstants::kBackLeftPosition, DriveConstants::kBackRightPosition}},
      pidX{0.9, 1e-4, 0}, pidY{0.9, 1e-4, 0}, pidRot{0.15, 0, 0},
      networkTableInst(nt::NetworkTableInstance::GetDefault()),
      m_poseEstimator{
          kSwerveKinematics,
          frc::Rotation2d(units::degree_t{m_pigeon.GetAngle()}),
          {modules[0].GetPosition(), modules[1].GetPosition(), modules[2].GetPosition(), modules[3].GetPosition()},
          frc::Pose2d()}
{
    navx.Calibrate();
    speeds = frc::ChassisSpeeds();
    networkTableInst.StartServer();
    frc::SmartDashboard::PutData("Field", &m_field);
    auto visionStdDevs = wpi::array<double, 3U>{0.9, 0.9, 1.8};
    m_poseEstimator.SetVisionMeasurementStdDevs(visionStdDevs);

    poseTable = networkTableInst.GetTable("ROS2Bridge");
    baseLink1Subscribe = poseTable->GetDoubleArrayTopic(baseLink1).Subscribe({}, {.periodic = 0.01, .sendAll = true});
    baseLink2Subscribe = poseTable->GetDoubleArrayTopic(baseLink2).Subscribe({}, {.periodic = 0.01, .sendAll = true});

    baseLinkPublisher = poseTable->GetDoubleArrayTopic(baseLink).Publish();

    // Configure Auto Swerve
    pathplanner::AutoBuilder::configureHolonomic(
        [this]() { return this->GetPose(); }, // Robot pose supplier
        [this](frc::Pose2d poseReset) {
            this->ResetPose(poseReset);
        }, // Method to reset odometry (will be called if your auto has a starting pose)
        [this]() {
            frc::SmartDashboard::PutNumber("ac drive/vx", this->GetPose().X().value());
            return this->getRobotRelativeSpeeds();
        }, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
        [this](frc::ChassisSpeeds speedsRelative) {
            this->Drive(speedsRelative);
        },                                        // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds
        pathplanner::HolonomicPathFollowerConfig( // HolonomicPathFollowerConfig, this should likely live in your
                                                  // Constants class
            pathplanner::PIDConstants(5, 0.0, 0.0), // Translation PID constants
            pathplanner::PIDConstants(5, 0.0, 0.0), // Rotation PID constants
            0.5_mps,                                // Max module speed, in m/s
            0.4_m, // Drive base radius in meters. Distance from robot center to furthest module.
            pathplanner::ReplanningConfig() // Default path replanning config. See the API for the options here
            ),
        []() {
            // Boolean supplier that controls when the path will be mirrored for the red alliance
            // This will flip the path being followed to the red side of the field.
            // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

            auto alliance = frc::DriverStation::GetAlliance();
            if (alliance)
            {
                return alliance.value() == frc::DriverStation::Alliance::kRed;
            }
            return false;
        },
        this // Reference to this subsystem to set requirements
    );
}

// This method will be called once per scheduler run
void SwerveDrive::Periodic()
{
    // getCameraResults();
    // sensor fusion? EKF (eek kinda fun) (extended Kalman filter)

    PublishOdometry(m_poseEstimator.GetEstimatedPosition());
    UpdatePoseEstimate();

    // PrintNetworkTableValues();

    frc::SmartDashboard::PutNumber("Heading", GetHeading().Degrees().value());
    // UpdateOdometry();
}

void SwerveDrive::Drive(frc::ChassisSpeeds speeds)
{
    if (enable)
    {

        auto states = kSwerveKinematics.ToSwerveModuleStates(speeds);

        kSwerveKinematics.DesaturateWheelSpeeds(&states, speeds, units::meters_per_second_t{ModuleConstants::kMaxSpeed},
                                                DriveConstants::kMaxTranslationalVelocity,
                                                DriveConstants::kMaxRotationalVelocity);

        for (int i = 0; i < 4; i++)
        {
            modules[i].SetDesiredState(states[i]);
        }

        frc::SmartDashboard::PutNumber("drive/vx", speeds.vx.value());
        frc::SmartDashboard::PutNumber("drive/vy", speeds.vy.value());
        frc::SmartDashboard::PutNumber("drive/omega", speeds.omega.value());
    }
}

void SwerveDrive::Strafe(frc::ChassisSpeeds s_speeds, double desiredAngle)
{
    auto currentAngle = m_poseEstimator.GetEstimatedPosition().Rotation().Radians().value();

    double errorBand = (M_PI - (-M_PI)) / 2;
    pos_Error = frc::InputModulus(desiredAngle - currentAngle, -errorBand, errorBand);

    s_speeds.omega = units::angular_velocity::radians_per_second_t{9.5 * (pos_Error)};
    frc::SmartDashboard::PutNumber("wrapped cA", currentAngle);
    frc::SmartDashboard::PutNumber("wrapped dA", desiredAngle);

    auto states = kSwerveKinematics.ToSwerveModuleStates(s_speeds);

    kSwerveKinematics.DesaturateWheelSpeeds(&states, s_speeds, units::meters_per_second_t{ModuleConstants::kMaxSpeed},
                                            DriveConstants::kMaxTranslationalVelocity,
                                            units::radians_per_second_t{0.5});

    for (int i = 0; i < 4; i++)
    {
        modules[i].SetDesiredState(states[i]);
    }

    priorSpeeds = s_speeds;
}

void SwerveDrive::SetFast()
{
}

void SwerveDrive::SetSlow()
{
}

frc::Rotation2d SwerveDrive::GetHeading()
{
    return m_pigeon.GetRotation2d();
}

void SwerveDrive::ResetHeading()
{
    if (enable == true)
    {
        m_pigeon.Reset();
    }
}

void SwerveDrive::ResetDriveEncoders()
{
    for (auto &module : modules)
    {
        module.ResetDriveEncoders();
    }
}

std::array<frc::SwerveModulePosition, 4> SwerveDrive::GetModulePositions()
{
    return std::array<frc::SwerveModulePosition, 4>{
        {modules[0].GetPosition(), modules[1].GetPosition(), modules[2].GetPosition(), modules[3].GetPosition()}};
}

void SwerveDrive::ResetPose(frc::Pose2d position)
{
    // odometry.ResetPosition(GetHeading(), GetModulePositions(), position);
    m_poseEstimator.ResetPosition(GetHeading(), GetModulePositions(), position);
}

frc::Pose2d SwerveDrive::GetPose()
{
    return m_poseEstimator.GetEstimatedPosition();
}

void SwerveDrive::UpdateOdometry()
{
    // odometry.Update(GetHeading(), GetModulePositions());
}
void SwerveDrive::SetVision()
{

    m_poseEstimator.ResetPosition(m_pigeon.GetRotation2d(), GetModulePositions(), GetVision());
}

frc::Pose2d SwerveDrive::GetVision()
{
    auto result2 = baseLink2Subscribe.GetAtomic();
    // auto time = result.time; // time stamp
    if (result2.value.size() > 0)
    {
        auto compressedResults = result2.value;
        rotation_q = frc::Quaternion(compressedResults.at(6), compressedResults.at(3), compressedResults.at(4),
                                     compressedResults.at(5));

        auto posTranslation =
            frc::Translation3d(units::meter_t{compressedResults.at(0)}, units::meter_t{compressedResults.at(1)},
                               units::meter_t{compressedResults.at(2)});
        frc::Pose3d cameraPose = frc::Pose3d(posTranslation, frc::Rotation3d(rotation_q));
        frc::Pose2d visionMeasurement2d = cameraPose.ToPose2d();
        return visionMeasurement2d;
    }
    else
    {

        return frc::Pose2d{};
    }
}

frc::ChassisSpeeds SwerveDrive::getRobotRelativeSpeeds()
{
    auto temp = kSwerveKinematics.ToChassisSpeeds({modules[0].GetCurrentState(), modules[1].GetCurrentState(),
                                                   modules[2].GetCurrentState(), modules[3].GetCurrentState()});

    return temp;
}

void SwerveDrive::InitializePID()
{
    pidX = frc::PIDController(0.9, 1e-4, 0);
    pidY = frc::PIDController(0.9, 1e-4, 0);
    pidRot = frc::PIDController(0.15, 0, 0);

    pidX.SetTolerance(0.025);
    pidY.SetTolerance(0.025);
    pidRot.SetTolerance(1);

    hasRun = false;
}

void SwerveDrive::SetReference(frc::Pose2d desiredPose)
{
    if ((!pidX.AtSetpoint() && !pidY.AtSetpoint()) | !hasRun)
    {
        speeds = frc::ChassisSpeeds{
            units::meters_per_second_t{pidX.Calculate(GetPose().X().value(), desiredPose.X().value())},
            units::meters_per_second_t{pidY.Calculate(GetPose().Y().value(), desiredPose.Y().value())},
            units::radians_per_second_t{0}};
        Drive(speeds);
    }
}

//--------------------------------------------

void SwerveDrive::UpdatePoseEstimate()
{
    auto result1 = baseLink1Subscribe.GetAtomic();
    auto result2 = baseLink2Subscribe.GetAtomic();
    // auto time = result.time; // time stamp

    if (result1.value.size() > 0 && useVision)
    {
        auto compressedResults = result1.value;
        rotation_q = frc::Quaternion(compressedResults.at(6), compressedResults.at(3), compressedResults.at(4),
                                     compressedResults.at(5));

        auto posTranslation =
            frc::Translation3d(units::meter_t{compressedResults.at(0)}, units::meter_t{compressedResults.at(1)},
                               units::meter_t{compressedResults.at(2)});
        frc::Pose3d cameraPose = frc::Pose3d(posTranslation, frc::Rotation3d(rotation_q));
        frc::Pose2d visionMeasurement2d = cameraPose.ToPose2d();
        m_poseEstimator.AddVisionMeasurement(visionMeasurement2d, units::second_t{compressedResults.at(7)});
    }
    if (result2.value.size() > 0 && useVision)
    {
        auto compressedResults = result2.value;
        rotation_q = frc::Quaternion(compressedResults.at(6), compressedResults.at(3), compressedResults.at(4),
                                     compressedResults.at(5));

        auto posTranslation =
            frc::Translation3d(units::meter_t{compressedResults.at(0)}, units::meter_t{compressedResults.at(1)},
                               units::meter_t{compressedResults.at(2)});
        frc::Pose3d cameraPose = frc::Pose3d(posTranslation, frc::Rotation3d(rotation_q));
        frc::Pose2d visionMeasurement2d = cameraPose.ToPose2d();
        m_poseEstimator.AddVisionMeasurement(visionMeasurement2d, units::second_t{compressedResults.at(7)});
    }

    m_poseEstimator.Update(m_pigeon.GetRotation2d(), GetModulePositions());
    m_field.SetRobotPose(m_poseEstimator.GetEstimatedPosition());
}

void SwerveDrive::PublishOdometry(frc::Pose2d odometryPose)
{
    double time = nt::Now() / (1e6);
    Eigen::Vector3d odoRotation = Eigen::Vector3d(0.0, 0.0, double(odometryPose.Rotation().Radians()));
    frc::Quaternion odoPoseQ = frc::Quaternion::FromRotationVector(odoRotation);
    double poseDeconstruct[]{double{odometryPose.X()},
                             double{odometryPose.Y()},
                             0.0,
                             odoPoseQ.X(),
                             odoPoseQ.Y(),
                             odoPoseQ.Z(),
                             odoPoseQ.W(),
                             time};
    baseLinkPublisher.Set(poseDeconstruct, time);
}

void SwerveDrive::EnableDrive()
{
    enable = true;
    // frc::SmartDashboard::PutBoolean("TestTestTest", enable);
}
void SwerveDrive::DisableDrive()
{
    enable = false;
    // frc::SmartDashboard::PutBoolean("TestTestTest", enable);
}
bool SwerveDrive::atSetpoint()
{
    if (pos_Error < 0.05)
    {
        return true;
    }
    return false;
}

void SwerveDrive::TurnVisionOff()
{
    useVision = false;
}

void SwerveDrive::TurnVisionOn()
{
    useVision = true;
}