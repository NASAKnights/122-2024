// Copyright (c) FRC Team 122. All Rights Reserved.

#pragma once
#include <cmath>
#include <cstdlib>
#include <numbers>

#include <ctre/phoenix6/controls/NeutralOut.hpp>
#include <ctre/phoenix6/signals/SpnEnums.hpp>
#include <frc/controller/SimpleMotorFeedforward.h>
#include <frc/geometry/Rotation2d.h>
#include <frc/geometry/Translation2d.h>
#include <frc/kinematics/SwerveDriveKinematics.h>
#include <frc/kinematics/SwerveModuleState.h>
#include <units/acceleration.h>
#include <units/angular_velocity.h>
#include <units/math.h>
#include <units/time.h>
#include <units/velocity.h>

#include "SDSModuleType.hpp"

namespace ElectricalConstants {

const int kFrontLeftDriveMotorID = 10;
const int kFrontLeftTurnMotorID = 11;
const int kFrontLeftEncoderID = 12;

const int kFrontRightDriveMotorID = 20;
const int kFrontRightTurnMotorID = 21;
const int kFrontRightEncoderID = 22;

const int kBackLeftDriveMotorID = 30;
const int kBackLeftTurnMotorID = 31;
const int kBackLeftEncoderID = 32;

const int kBackRightDriveMotorID = 40;
const int kBackRightTurnMotorID = 41;
const int kBackRightEncoderID = 42;

} // namespace ElectricalConstants

namespace DriveConstants {

const int kDriverPort = 0;
const int kOperatorPort = 1;

const SDSModuleType mk4i_l1{0.10033,
                            (14.0 / 50.0) * (25.0 / 19.0) * (15.0 / 45.0), true,
                            (14.0 / 50.0) * (10.0 / 60.0), false};
const SDSModuleType mk4i_l2{0.10033,
                            (14.0 / 50.0) * (27.0 / 17.0) * (15.0 / 45.0), true,
                            (14.0 / 50.0) * (10.0 / 60.0), false};
const SDSModuleType mk4i_l3{0.10033,
                            (14.0 / 50.0) * (28.0 / 16.0) * (15.0 / 45.0), true,
                            (14.0 / 50.0) * (10.0 / 60.0), false};

const SDSModuleType kSDSModule = mk4i_l3;

const auto kTrackwidthMeters = 0.4_m;
const auto kWheelbaseMeters = 0.4_m;

const double kDefaultAxisDeadband = 0.15;
const units::meters_per_second_t kMaxTranslationalVelocity{0.5};

const units::radians_per_second_t kMaxRotationalVelocity{1};
const bool kIsFieldRelative = true;

const frc::Rotation2d kFrontLeftOffset{-units::degree_t{103}}; // module 1
const frc::Rotation2d kFrontRightOffset{
    -units::degree_t{-40 - 2.5}}; // 139.658 // 139.658 // module 2
const frc::Rotation2d kBackLeftOffset{
    -units::degree_t{157 + 3.6}}; // module 3
const frc::Rotation2d kBackRightOffset{
    -units::degree_t{82 + 3}}; // 265.517 // -93.867 // module 4

const frc::Translation2d kFrontLeftPosition =
    frc::Translation2d(units::meter_t{kTrackwidthMeters / 2.0},
                       units::meter_t{kWheelbaseMeters / 2.0});
const frc::Translation2d kFrontRightPosition =
    frc::Translation2d(units::meter_t{kTrackwidthMeters / 2.0},
                       units::meter_t{-kWheelbaseMeters / 2.0});
const frc::Translation2d kBackLeftPosition =
    frc::Translation2d(units::meter_t{-kTrackwidthMeters / 2.0},
                       units::meter_t{kWheelbaseMeters / 2.0});
const frc::Translation2d kBackRightPosition =
    frc::Translation2d(units::meter_t{-kTrackwidthMeters / 2.0},
                       units::meter_t{-kWheelbaseMeters / 2.0});

const frc::SwerveDriveKinematics kSwerveKinematics =
    frc::SwerveDriveKinematics(kFrontLeftPosition, kFrontRightPosition,
                               kBackLeftPosition, kBackRightPosition);

const double kDriveLimit = 0.15; // 0.7 fast
const double kRotationLimit = kDriveLimit;

} // namespace DriveConstants

namespace ModuleConstants {

// meters / second
const auto kMaxSpeed = DriveConstants::kMaxTranslationalVelocity;
// meters
const auto kWheelDiameterMeters =
    units::meter_t{0.092815210491};
// meters / turn
const auto kWheelCircumference =
    kWheelDiameterMeters * std::numbers::pi / units::turn_t{1.0};
// ratio is motor rot / wheel rot
const double kDriveGearRatio = 1.0 / DriveConstants::kSDSModule.driveReduction;
const double kTurnGearRatio = 1.0 / DriveConstants::kSDSModule.steerReduction;

const auto kDriveConversion = kWheelCircumference / kDriveGearRatio;

const bool kDriveMotorInverted = DriveConstants::kSDSModule.driveInverted;

const bool kSteerMotorInverted = true;

const auto kDriveMotorNeutral =
    ctre::phoenix6::signals::NeutralModeValue::Brake;
const auto kSteerMotorNeutral =
    ctre::phoenix6::signals::NeutralModeValue::Coast; // set back to brake to be
                                                      // amazing
const bool kEncoderInverted = false;

const bool kSteerEnableCurrentLimit = true;
const int kSteerContinuousCurrentLimit = 25;
const int kSteerPeakCurrentLimit = 40;
const double kSteerPeakCurrentDuration = 0.1;

const bool kDriveEnableCurrentLimit = true;
const int kDriveContinuousCurrentLimit = 35;
const int kDrivePeakCurrentLimit = 60;
const double kDrivePeakCurrentDuration = 0.1;

// TODO: retune constants
const double kDriveP = 0.2;
const double kDriveI = 0.0;
const double kDriveD = 0.0;
const double kDriveS = 0.02496863326; // Volts
const double kDriveV = 0.1089791826;  // Volts / (rot / s)
const double kDriveA = 0.0;           // Volts / (rot / s^2)

// const double kDriveS = 0.05558; // Volts
// const double kDriveV = 0.20333; // Volts / (rot / s)
// const double kDriveA = 0.02250; // Volts / (rot / s^2)

const double kSteerP = 35.0; // TODO: ensure this works with new inversion
const double kSteerI = 0.0;
const double kSteerD = 0.0;

} // namespace ModuleConstants

namespace MathUtilNK {
inline double calculateAxis(double axis, double deadband) {
  if (std::abs(axis) > deadband) {
    return (axis - std::copysign(deadband, axis)) / (1.0 - deadband);
  } else {
    return 0.0;
  }
}

} // namespace MathUtilNK
