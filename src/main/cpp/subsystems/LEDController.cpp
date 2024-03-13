// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/LEDController.h"


LEDController::LEDController() 
{
    candle.ConfigLEDType(ctre::phoenix::led::LEDStripType::GRB);
    candle.SetLEDs(0, 0, 0);
}

void LEDController::DefaultAnimation()
{
    for(int i = 0; i < 5; i++)
    {
        candle.ClearAnimation(i);
    }
    auto rgbfade = ctre::phoenix::led::RgbFadeAnimation(1.0, 0.7, -1, 8);
    auto rainbow = ctre::phoenix::led::RainbowAnimation(1.0, 0.7, -1, false, 8);
    auto fire = ctre::phoenix::led::FireAnimation(1.0, 0.7, -1, 1, 1, false, 8);
    auto twinkle = ctre::phoenix::led::TwinkleAnimation(0,0,255,1, 0.7, -1, ctre::phoenix::led::TwinkleAnimation::Percent100, 8);
    candle.Animate(rainbow, 0);
}

void LEDController::Periodic() {}

void LEDController::HandleIntakeState() {
    if(m_intakeState != m_intakeStatePrev)
    {
        switch (m_intakeState)
        {
        case NO_NOTE:
            candle.SetLEDs(255, 0, 0);
            break;
        
        case NOTE:
            candle.SetLEDs(0, 255, 0);
            break;

        default:
            candle.SetLEDs(255, 255, 255);
            break;
        }
    }
    m_intakeStatePrev = m_intakeState;
}

void LEDController::HandleShooterState(){
    if(m_shooterState != m_shooterStatePrev)
    {
        switch (m_shooterState)
        {
        case LED_NO_SHOT:
            candle.SetLEDs(255, 0, 0); //Red
            break;
        
        case LED_SPIN_UP:
            candle.SetLEDs(255, 255, 0); //Yellow
            break;

        case LED_SHOOTING:
            ledGroup1.SetLarson(255, 30, 0, 4);
            ledGroup2.SetLarson(255, 30, 0, 3);
            ledGroup4.SetLarson(255, 30, 0, 3);
            ledGroup5.SetLarson(255, 30, 0, 3);
            break;
        case LED_BAD:
            AllLEDs.SetFlash(255, 0, 0, 255, 30, 0, Time);
            break;
        default:
            candle.SetLEDs(255, 255, 255);
            break;
        }
    }
    m_shooterStatePrev = m_shooterState;
}
