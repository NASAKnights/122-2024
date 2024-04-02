// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/SubsystemBase.h>
#include <ctre/phoenix/led/CANdle.h>
#include <frc/DigitalInput.h>
#include <frc2/command/button/Trigger.h>
#include "subsystems/LED_Groups.h"

enum LEDIntakeState
{
  NO_NOTE,  //Red
  NOTE  //Green
};

enum LEDShooterState
{
  LED_NO_SHOT,  // Orange
  LED_SPIN_UP,  // Yellow
  LED_SHOOTING,  //Green
  LED_BAD  //Blinking Red
};

class LEDController : public frc2::SubsystemBase {
 public:
  LEDController();
  void HandleIntakeState();
  void HandleShooterState();
  void DefaultAnimation();
  /**
   * Will be called periodically whenever the CommandScheduler runs.
   */
  void Periodic() override;

  LEDIntakeState m_intakeState = LEDIntakeState::NO_NOTE;
  LEDShooterState m_shooterState = LEDShooterState::LED_BAD;
 ctre::phoenix::led::CANdle candle {9};

private:
  LEDIntakeState m_intakeStatePrev = LEDIntakeState::NO_NOTE;
  LEDShooterState m_shooterStatePrev = LEDShooterState::LED_BAD;

 units::time::second_t Time{0.2};
 units::time::second_t Speed{0.1};
  
 int _r = 255;
 int _g = 0;
 int _b = 0;
 int i = 0;

 bool P_state;
 bool C_state;

 std::vector<int> group1 = {8,9,10,11,12,13,14,15,16,17,18,19,20,21,22,23,24,25,26,27,28,29,30,31,32,33};
 LED_Group ledGroup1 = LED_Group(&candle, group1, 0);
 
 std::vector<int> group2 = {34,35,36,37,38,39,40,41,42,43,44,45,46,47,48,49,50,51,52,53,54,55,56,57,58,59};
 LED_Group ledGroup2 = LED_Group(&candle, group2, 1);

 std::vector<int> group3 = {60,61,62,63,64,65,66,67,68,69,70,71,72,73,74,75,76,77,78};
 LED_Group ledGroup3 = LED_Group(&candle, group3, 2);

 std::vector<int> group4 = {79,80,81,82,83,84,85,86,87,88,89,90,91,92,93,94,95,96,97,98,99,100,101,102,103,104};
 LED_Group ledGroup4 = LED_Group(&candle, group4, 3);

  std::vector<int> group5 = {105,106,107,108,109,110,111,112,113,114,115,116,117,118,119,120,121,122,123,124,125,126,127,128,129,130};
 LED_Group ledGroup5 = LED_Group(&candle, group5, 4);

  std::vector<int> AllLeds = {8,9,10,11,12,13,14,15,16,17,18,19,20,21,22,23,24,25,26,27,28,29,30,31,32,33,34,35,36,37,38,39,40,41,42,43,44,45,46,47,48,49,50,51,52,53,54,55,56,57,58,59,60,61,62,63,64,65,66,67,68,69,70,71,72,73,74,75,76,77,78,79,80,81,82,83,84,85,86,87,88,89,90,91,92,93,94,95,96,97,98,99,100,101,102,103,104,105,106,107,108,109,110,111,112,113,114,115,116,117,118,119,120,121,122,123,124,125,126,127,128,129,130,131};
 LED_Group AllLEDs = LED_Group(&candle, AllLeds, 5);


};