// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/Shooter.h"
#include <frc/smartdashboard/SmartDashboard.h>

namespace ShooterConstants {
    const double kShootP = 0.02;
    const double kShootI = 0.0;
    const double kShootD = 0.0;
    // const double kShootS = 0.02496863326; //KEEP IF DOING FF
    // const double kShootV = 0.1089791826;
    // const double kShootA = 0.2;
 
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

    m_shootMotorTop.SetSmartCurrentLimit(30);
    m_shootMotorBot.SetSmartCurrentLimit(30);

    m_shootMotorTop.EnableVoltageCompensation(12.0);
    m_shootMotorBot.EnableVoltageCompensation(12.0);
    
    m_shootMotorTop.SetIdleMode(rev::CANSparkMax::IdleMode::kCoast);
    m_shootMotorBot.SetIdleMode(rev::CANSparkMax::IdleMode::kCoast);

    m_shootMotorBot.SetInverted(true);
    
}

void Shooter::Periodic() {
}


void Shooter::Shoot(double shootSpeed) {
    frc::SmartDashboard::PutNumber("Shooter Velocity", m_shootEncoderTop.GetVelocity());
    frc::SmartDashboard::PutNumber("Shooter goal Vel", shootSpeed * 11000);
    
    m_shootMotorTop.Set(-shootSpeed);
    m_shootMotorBot.Set(shootSpeed);
    
    running = true;
    SHOOT_speed = shootSpeed;
}

void Shooter::stopShooter() {

    m_shootMotorTop.Set(0.0);
    m_shootMotorBot.Set(0.0);
    
    running = false;
}

double Shooter::getSpeed() {
    return m_shootEncoderTop.GetVelocity();
}

double Shooter::getShuffleGoal() {
    return SHOOT_speed;
}

bool Shooter::atSetpoint() {
    if(fabs(getSpeed()) >= fabs(SHOOT_speed*11000.0)) // Rpm
    {
        return true;
    }
    return false;
}

bool Shooter::isRunning() {
    return running;
}