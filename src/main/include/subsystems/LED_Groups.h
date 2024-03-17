// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

// #include "LEDController.h"
#include <vector>
#include <ctre/phoenix/led/CANdle.h>
#include <frc2/command/WaitCommand.h>
#include <ctre/phoenix/led/Animation.h>
#include <ctre/phoenix/led/RgbFadeAnimation.h>
#include <ctre/phoenix/led/RainbowAnimation.h>
#include <ctre/phoenix/led/FireAnimation.h>
#include <ctre/phoenix/led/ColorFlowAnimation.h>
#include <ctre/phoenix/led/TwinkleAnimation.h>
#include <ctre/phoenix/led/LarsonAnimation.h>
#include <frc/smartdashboard/SmartDashboard.h>

class LED_Group{
 public:
  LED_Group(ctre::phoenix::led::CANdle* candle, std::vector<int> group, int slotID) :
         _candle(candle),
         _group(group),
         _slotID(slotID)
         {          
         }
  
  void SetColor(int _r, int _g, int _b);
  void SetLarson(int _r, int _g, int _b, int Length);
  void SetDisco(int _r, int _g, int _b);
  void SetFlash(int _r, int _g, int _b, int _R, int _G, int _B, units::time::second_t Period);
  void SetGradient(int _r, int _g, int _b, int r, int g, int b);
  void SetScrollingGradient(int _r, int _g, int _b, int r, int g, int b);
  void SetRainbow();
  std::vector<std::vector<int>> ComputeGradent(int _r, int _g, int _b, int _R, int _G, int _B);
    /**
   * Will be called periodically whenever the CommandScheduler runs.
   */
 private:

  ctre::phoenix::led::CANdle* _candle;
  std::vector<int> _group;
  int random;
  int LarsonFront = 0;
  int StepR;
  int StepG;
  int StepB;
  int Offset = 8;
  int ROffset;
  int _start = 0;
  int _slotID = 0;
  bool _status = false;

  frc::Timer _timer;

  ctre::phoenix::led::RgbFadeAnimation rgbFade;
  
};
