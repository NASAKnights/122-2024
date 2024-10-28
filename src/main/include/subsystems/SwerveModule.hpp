// Copyright (c) FRC Team 122. All Rights Reserved.

#pragma once

#include <ctre/phoenix6/CANcoder.hpp>
#include <ctre/phoenix6/TalonFX.hpp>
#include <frc/controller/SimpleMotorFeedforward.h>
#include <frc/geometry/Rotation2d.h>
#include <frc/kinematics/SwerveModulePosition.h>
#include <frc/kinematics/SwerveModuleState.h>
#include <frc/simulation/SimDeviceSim.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc2/command/SubsystemBase.h>
#include <units/math.h>
#include <units/velocity.h>
#include <units/voltage.h>

#include "Constants.hpp"

class SwerveModule : public frc2::SubsystemBase
{
  public:
    SwerveModule(int driveMotorId, int steerMotorId, int steerEncoderId, frc::Rotation2d angleOffset);

    /**
     * Will be called periodically whenever the CommandScheduler runs.
     */
    void Periodic() override;

    /**
     * Will be called perodically by the scheduler when simulation runs.
     */
    void SimulationPeriodic() override;

    frc::SwerveModuleState GetCurrentState();
    frc::SwerveModulePosition GetPosition();
    void SetDesiredState(frc::SwerveModuleState desiredState);
    void ResetDriveEncoders();
    void InitEncoder(int encoderID);
    frc::Rotation2d GetRotation();
    frc::Rotation2d GetAbsoluteRotation();
    void Reset();

  private:
    // Components (e.g. motor controllers and sensors) should generally be
    // declared private and exposed only through public methods.
    int m_id;

    ctre::phoenix6::hardware::TalonFX m_driveMotor;
    ctre::phoenix6::hardware::TalonFX m_steerMotor;
    ctre::phoenix6::hardware::CANcoder m_steerEncoder;

    frc::Rotation2d m_angleOffset;

    // simulation fields
    frc::Timer m_simTimer;

    frc::sim::SimDeviceSim m_driveSim;
    frc::sim::SimDeviceSim m_steerSim;

    hal::SimDouble m_driveSimVelocity;
    hal::SimDouble m_driveSimPosition;
    hal::SimDouble m_steerSimPosition;
};
