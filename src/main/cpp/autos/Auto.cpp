// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "autos/Auto.h"

// NOTE:  Consider using this command inline, rather than writing a subclass.
// For more information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
Auto::Auto(SwerveDrive* m_swerve,
            Shooter* shooter, Indexer* indexer, Intake* intaker,
            ArmSubsystem* armer, int autoName, LEDController* led_controller)
            {
  // Add your commands here, e.g.
  // AddCommands(FooCommand{}, BarCommand{});
  switch(autoName){
  case 1:
    AddCommands(
      TrajectoryFollower(m_swerve, &NKTrajectoryManager::GetTrajectory("Red_Near_Loading")),
      Shoot(shooter, indexer, intaker, armer, led_controller, 120, 78));
    break;
  case 2:
    AddCommands(
      TrajectoryFollower(m_swerve, &NKTrajectoryManager::GetTrajectory("Blue_Near_Loading")),
      Shoot(shooter, indexer, intaker, armer, led_controller, 120, 78));
      break;
  }
}

frc2::CommandHelper<frc2::SequentialCommandGroup,Auto> Blue_Middle(){
  
}
