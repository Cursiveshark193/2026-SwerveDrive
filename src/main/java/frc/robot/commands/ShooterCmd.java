package frc.robot.commands; // package for robot subsystem classes

import edu.wpi.first.wpilibj2.command.Command; // WPILib Command type for commands
import edu.wpi.first.wpilibj.Timer; // WPILib Timer for timing operations

import frc.robot.subsystems.ShooterSubsystem; // shooter subsystem class

public class ShooterCmd extends Command {

    private final ShooterSubsystem shooterSubsystem;
    private double stopTime;
    private Timer timer;

    public ShooterCmd(ShooterSubsystem shooterSubsystem, double stopTime){
        this.shooterSubsystem = shooterSubsystem;
        this.stopTime = stopTime;
        this.timer = new Timer();
      
        addRequirements(shooterSubsystem);
    }




    // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    timer.reset(); // reset the timer to start from 0
    timer.start(); // start the timer when the command is initialized

  }



  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    shooterSubsystem.setDutyCycle(1.0);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    shooterSubsystem.setDutyCycle(0.0); // stop the shooter when the command ends
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return timer.hasElapsed(stopTime); // end the command after the specified stop time has elapsed
     
  }
}