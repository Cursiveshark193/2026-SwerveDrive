// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.ShooterSubsystem;
//import frc.robot.subsystems.FeederSubsystem;
import frc.robot.subsystems.conveyor;
import edu.wpi.first.wpilibj2.command.Command;

/** Utility class containing autonomous command factories. */
public final class Autos {
  /**
   * Example static factory for an autonomous command.
   *
   * @param shooter the shooter subsystem
   * @param feeder the feeder subsystem
   * @param conv the conveyor subsystem
   * @param drivbase 
   * @return an example autonomous command sequence
   *///FeederSubsystem feeder,
  public static Command exampleAuto(ShooterSubsystem shooter,  conveyor conv) {
    // Return a simple ExampleCommand that requires the shooter, feeder, and conveyor.
    return new RunShooterFeederConveyor(shooter, feeder, conv);
    
    return new RunShooterFeederConveyor(shooter, 
    //feeder, 
    conv);

  }

  private Autos() {
    throw new UnsupportedOperationException("This is a utility class!");
  }
}
