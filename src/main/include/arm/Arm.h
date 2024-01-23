// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/SubsystemBase.h>

class Arm : public frc2::SubsystemBase {
 public:
  Arm();


  void Periodic() override;

  // move arm in ()
  // move arm out()

  // print () (optional) - encoder value

 private:

  // one motor (assume TalonFX)
  // assume relative encoder 
  // can easily change to absolute

  // assume gearing is 1:1
  // might? two limit switches


};
