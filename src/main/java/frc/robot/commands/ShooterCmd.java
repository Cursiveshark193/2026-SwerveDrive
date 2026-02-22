package frc.robot.commands; // package for robot subsystem classes

import static edu.wpi.first.units.Units.Amps; // unit helper for current
import static edu.wpi.first.units.Units.Inches; // unit helper for inches
import static edu.wpi.first.units.Units.Pounds; // unit helper for mass
import static edu.wpi.first.units.Units.RPM; // unit helper for RPM
import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond; // unit helper for angular accel
import static edu.wpi.first.units.Units.Second; // unit helper for seconds
import static edu.wpi.first.units.Units.Seconds; // plural seconds helper
import static edu.wpi.first.units.Units.Volts; // unit helper for volts

import com.ctre.phoenix6.CANBus; // CTRE CAN bus type (unused but often available)
import com.ctre.phoenix6.hardware.TalonFX; // CTRE TalonFX motor controller class
import com.revrobotics.spark.SparkMax; // REV SparkMax (import present though not used here)

import edu.wpi.first.math.system.plant.DCMotor; // WPILib DCMotor model used for simulation
import edu.wpi.first.units.measure.AngularVelocity; // units type for angular velocity
import edu.wpi.first.wpilibj2.command.Command; // WPILib Command type for commands
import edu.wpi.first.wpilibj2.command.SubsystemBase; // base class for subsystems

import java.util.function.Supplier; // functional interface used for supplier-based commands

import edu.wpi.first.math.Pair; // simple Pair utility used by YAMS config
import edu.wpi.first.math.controller.SimpleMotorFeedforward; // feedforward helper
import yams.gearing.GearBox; // gearbox helper from YAMS
import yams.gearing.MechanismGearing; // mechanism gearing wrapper
import yams.mechanisms.config.FlyWheelConfig; // config for FlyWheel mechanism
import yams.mechanisms.velocity.FlyWheel; // FlyWheel mechanism class
import yams.motorcontrollers.SmartMotorController; // YAMS abstraction for motor controllers
import yams.motorcontrollers.SmartMotorControllerConfig; // YAMS motor controller configuration
import yams.motorcontrollers.SmartMotorControllerConfig.ControlMode; // control mode enum
import yams.motorcontrollers.SmartMotorControllerConfig.MotorMode; // motor idle mode enum
import yams.motorcontrollers.SmartMotorControllerConfig.TelemetryVerbosity; // telemetry verbosity enum
import yams.motorcontrollers.remote.TalonFXWrapper; // wrapper to adapt TalonFX to SmartMotorController
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