// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import static edu.wpi.first.units.Units.RPM;

import frc.robot.subsystems.FeederSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.conveyor;

public class RunShooterFeederConveyor extends ParallelCommandGroup {
  /**
   * Run shooter, feeder, and conveyor simultaneously.
   *
   * Replace the feeder/conveyor command calls with the actual command factories.
   * 
   * @param shooter the shooter subsystem to run
   * @param feeder the feeder subsystem to run
   * @param conv the conveyor subsystem to run
   */ 
  public RunShooterFeederConveyor(ShooterSubsystem shooter, FeederSubsystem feeder, conveyor conv) {
    // Add the commands to run in parallel. These must be Commands returned by your subsystems.
    addCommands(
      shooter.setVelocity(RPM.of(4000)),       // closed-loop shooter command
      feeder.ReverseFeeder(),                  // <-- replace with real feeder command
      conv.ReverseConveyor()           // <-- replace with real conveyor command
    );

    // Declare that this command requires all three subsystems to avoid resource conflicts.
    addRequirements(shooter, feeder, conv);
  }
}
