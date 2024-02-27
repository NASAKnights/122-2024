// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/Shooter.h"
#include <frc/smartdashboard/SmartDashboard.h>

namespace ShooterConstants {
    const double kShootP = 0.02;
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
    m_shootMotorTop{6, rev::CANSparkMaxLowLevel::MotorType::kBrushless},
    m_shootMotorBot{7, rev::CANSparkMaxLowLevel::MotorType::kBrushless}
{ 
    
    m_shootPIDTop.SetP(ShooterConstants::kShootP);
    m_shootPIDTop.SetI(ShooterConstants::kShootI);
    m_shootPIDTop.SetD(ShooterConstants::kShootD);
    m_shootPIDBot.SetP(ShooterConstants::kShootP);
    m_shootPIDBot.SetI(ShooterConstants::kShootI);
    m_shootPIDBot.SetD(ShooterConstants::kShootD);

    m_shootMotorTop.SetSecondaryCurrentLimit(35);
    m_shootMotorBot.SetSecondaryCurrentLimit(35);

    m_shootMotorTop.SetSmartCurrentLimit(30);
    m_shootMotorBot.SetSmartCurrentLimit(30);
    
    m_shootMotorTop.SetIdleMode(rev::CANSparkMax::IdleMode::kCoast);
    m_shootMotorBot.SetIdleMode(rev::CANSparkMax::IdleMode::kCoast);

    m_shootMotorBot.SetInverted(true);
    

}

void Shooter::Periodic() {
    // SHOOT_speed = frc::SmartDashboard::GetNumber("Shooter Speed", 0);
}


void Shooter::Shoot(double shootSpeed) {
    frc::SmartDashboard::PutNumber("Shooter Velocity", m_shootEncoderTop.GetVelocity());
    // m_shootPIDTop.SetReference(shootSpeed, rev::CANSparkBase::ControlType::kVelocity);
    // m_shootPIDBot.SetReference(shootSpeed, rev::CANSparkBase::ControlType::kVelocity);
    m_shootMotorTop.Set(-0.72);
    m_shootMotorBot.Set(0.7);
    
    running = true;
    SHOOT_speed = shootSpeed;
}

void Shooter::stopShooter() {

    m_shootMotorTop.Set(0.0);
    m_shootMotorBot.Set(0.0);
    
    running = false;
}

double Shooter::getSpeed() {
    // return double {m_shooterMotorMain.GetVelocity().GetValue()};
    return m_shootEncoderTop.GetVelocity();
}

double Shooter::getShuffleGoal() {
    return SHOOT_speed;
}

bool Shooter::atSetpoint() {
    if(fabs(getSpeed() - SHOOT_speed) < 10.0) // Rot/s
    {
        return true;
    }
    return false;
}

bool Shooter::isRunning() {
    return running;
}