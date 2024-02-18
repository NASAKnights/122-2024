// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/Shooter.h"

namespace ShooterConstants {
    const double kShootP = 0.2;
    const double kShootI = 0.0;
    const double kShootD = 0.0;
    const double kShootS = 0.02496863326;
    const double kShootV = 0.1089791826;
    const double kShootA = 0.2;

    const double motorRampSpeed = 5000; 

    ctre::phoenix6::signals::NeutralModeValue kShootMotorNeutral = 
        ctre::phoenix6::signals::NeutralModeValue::Brake; 
}


Shooter::Shooter() :
    m_follower(m_shooterMotorMain.GetDeviceID(), false),
    velocityControl(units::turns_per_second_t {0.0})
{ 
    
    ctre::phoenix6::configs::TalonFXConfiguration shooterConfig{};

    ctre::phoenix6::configs::Slot0Configs shootSlot0Configs{};
    shootSlot0Configs.kP = ShooterConstants::kShootP;
    shootSlot0Configs.kI = ShooterConstants::kShootI;
    shootSlot0Configs.kD = ShooterConstants::kShootD;
    shootSlot0Configs.kS = ShooterConstants::kShootS;
    shootSlot0Configs.kV = ShooterConstants::kShootV;
    shootSlot0Configs.kA = ShooterConstants::kShootA;
    
    shooterConfig.Slot0 = shootSlot0Configs;

    ctre::phoenix6::configs::CurrentLimitsConfigs shootCurrentLimitConfig{};
    shootCurrentLimitConfig.SupplyCurrentLimitEnable = true;
    shootCurrentLimitConfig.SupplyCurrentLimit = 35;
    shootCurrentLimitConfig.SupplyCurrentThreshold = 40;
    shootCurrentLimitConfig.SupplyTimeThreshold = 0.1;

    shooterConfig.CurrentLimits = shootCurrentLimitConfig;
    m_shooterMotorMain.SetNeutralMode(ShooterConstants::kShootMotorNeutral);
    m_shooterMotorMain.SetInverted(false);
    m_shooterMotorFollow.SetNeutralMode(ShooterConstants::kShootMotorNeutral);

    m_shooterMotorMain.GetConfigurator().Apply(shooterConfig);

    m_shooterMotorFollow.SetControl(m_follower);


}

void Shooter::Periodic() {}

void Shooter::Shoot() {
    m_shooterMotorMain.SetControl(
        velocityControl.WithVelocity(units::turns_per_second_t {-(5600.0 / 60.0)*0.6 }));
    running = true;
}

void Shooter::stopShooter() {
    m_shooterMotorMain.SetControl(
        velocityControl.WithVelocity(units::turns_per_second_t {0.0}));
    running = false;
}

double Shooter::getSpeed() {
    return double {m_shooterMotorMain.GetVelocity().GetValue()};
}

bool Shooter::isRunning() {
    return running;
}