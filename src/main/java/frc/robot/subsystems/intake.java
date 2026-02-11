// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems; // package containing robot subsystem classes

import static edu.wpi.first.units.Units.Amps; // unit helper for currents
import static edu.wpi.first.units.Units.DegreesPerSecond; // unit helper for angular velocity (deg/s)
import static edu.wpi.first.units.Units.DegreesPerSecondPerSecond; // angular acceleration (deg/s^2)
import static edu.wpi.first.units.Units.Inches; // unit helper for lengths
import static edu.wpi.first.units.Units.Pounds; // unit helper for mass
import static edu.wpi.first.units.Units.RPM; // unit helper for rotations per minute
import com.revrobotics.spark.SparkLowLevel.MotorType; // SparkMax motor type enum
import com.revrobotics.spark.SparkMax; // REV SparkMax controller class

import edu.wpi.first.math.controller.SimpleMotorFeedforward; // feedforward helper class used by controllers
import edu.wpi.first.wpilibj2.command.Command; // WPILib command type

import edu.wpi.first.math.system.plant.DCMotor; // WPILib motor model for sim
import edu.wpi.first.units.measure.AngularVelocity; // type for angular velocity units
import yams.motorcontrollers.SmartMotorController; // YAMS motor controller abstraction
import yams.motorcontrollers.local.SparkWrapper; // wrapper to adapt SparkMax to SmartMotorController
import edu.wpi.first.wpilibj2.command.SubsystemBase; // base class for subsystems
import yams.gearing.GearBox; // gearbox helper
import yams.mechanisms.config.FlyWheelConfig; // flywheel mechanism config
import yams.mechanisms.velocity.FlyWheel; // flywheel mechanism
import yams.gearing.MechanismGearing; // mechanism gearing wrapper
import yams.motorcontrollers.SmartMotorControllerConfig; // motor controller config builder
import yams.motorcontrollers.SmartMotorControllerConfig.ControlMode; // control mode enum
import yams.motorcontrollers.SmartMotorControllerConfig.MotorMode; // motor idle mode enum
import yams.motorcontrollers.SmartMotorControllerConfig.TelemetryVerbosity; // telemetry verbosity enum

/**
 * Intake subsystem that controls the intake motor and its simulated
 * mechanism.
 *
 * <p>This class wraps a REV SparkMax with a YAMS {@code SmartMotorController}
 * and exposes a {@link yams.mechanisms.velocity.FlyWheel} based mechanism for
 * simple velocity and open-loop control commands. Telemetry and simulation are
 * provided by the YAMS components.
 */
public class intake extends SubsystemBase { // intake subsystem for ball handling

  private SmartMotorControllerConfig smcConfig = new SmartMotorControllerConfig(this) // config builder for
                                                                                      // SmartMotorController
      .withControlMode(ControlMode.CLOSED_LOOP) // use closed-loop control by default
      // Feedback Constants (PID Constants)
      .withClosedLoopController(0, 0, 0, DegreesPerSecond.of(30), DegreesPerSecondPerSecond.of(45)) // PID constants +
                                                                                                    // safety
                                                                                                    // velocity/accel
                                                                                                    // limits
      .withSimClosedLoopController(0, 0, 0, DegreesPerSecond.of(30), DegreesPerSecondPerSecond.of(45)) // simulated PID
                                                                                                       // defaults
      // Feedforward Constants
      .withFeedforward(new SimpleMotorFeedforward(0, 0, 0)) // real robot feedforward gains
      .withSimFeedforward(new SimpleMotorFeedforward(0, 0, 0)) // sim feedforward gains
      // Telemetry name and verbosity level
      .withTelemetry("intakeMotor", TelemetryVerbosity.HIGH) // telemetry label for motor
      // Gearing from the motor rotor to final shaft.
      // In this example GearBox.fromReductionStages(3,4) is the same as
      // GearBox.fromStages("3:1","4:1") which corresponds to the gearbox attached to
      // your motor.
      // You could also use .withGearing(12) which does the same thing.
      .withGearing(new MechanismGearing(GearBox.fromReductionStages(3, 4))) // mechanism gearing config
      // Motor properties to prevent over currenting.
      .withMotorInverted(false) // motor inversion setting
      .withIdleMode(MotorMode.COAST) // idle/coast behavior
      .withStatorCurrentLimit(Amps.of(40)); // stator current limit to protect hardware
  // Vendor motor controller object
  private SparkMax spark = new SparkMax(16, MotorType.kBrushless); // create SparkMax on CAN ID 16

  // Create our SmartMotorController from our Spark and config with the NEO.
  private SmartMotorController sparkSmartMotorController = new SparkWrapper(spark, DCMotor.getNeo550(1), smcConfig); // wrap
                                                                                                                     // SparkMax
                                                                                                                     // with
                                                                                                                     // YAMS
                                                                                                                     // SmartMotorController

  private final FlyWheelConfig intakeConfig = new FlyWheelConfig(sparkSmartMotorController) // mechanism config for
                                                                                            // intake flywheel
      // Diameter of the flywheel.
      .withDiameter(Inches.of(4)) // set flywheel diameter
      // Mass of the flywheel.
      .withMass(Pounds.of(12)) // set flywheel mass
      // Maximum speed of the shooter.
      .withUpperSoftLimit(RPM.of(200)) // set upper soft speed limit
      // Telemetry name and verbosity for the arm.
      .withTelemetry("intakeMech", TelemetryVerbosity.HIGH); // telemetry label for mechanism

  // Shooter Mechanism
  private FlyWheel intake = new FlyWheel(intakeConfig); // create FlyWheel mechanism instance

  /**
   * Gets the current velocity of the shooter.
   *
   * @return Shooter velocity.
   */
  public AngularVelocity getVelocity() {
    return intake.getSpeed();
  }

  /**
   * Set the shooter velocity.
   *
   * @param speed Speed to set.
   * @return {@link edu.wpi.first.wpilibj2.command.RunCommand}
   */
  public Command setVelocity(AngularVelocity speed) {
    return intake.setSpeed(speed);
  }

  /**
   * Set the dutycycle of the shooter.
   *
   * @param dutyCycle DutyCycle to set.
   * @return {@link edu.wpi.first.wpilibj2.command.RunCommand}
   */
  public Command set(double dutyCycle) {
    return intake.set(dutyCycle);
  }

  /** Create a new Intake subsystem. */
  public intake() {
  }

  /**
   * Example command factory method.
   *
   * @return a command
   */
  public Command exampleMethodCommand() {
    // Inline construction of command goes here.
    // Subsystem::RunOnce implicitly requires `this` subsystem.
    return runOnce(
        () -> {
          /* one-time action goes here */
        });
  }

  /**
   * An example method querying a boolean state of the subsystem (for example, a
   * digital sensor).
   *
   * @return value of some boolean subsystem state, such as a digital sensor.
   */
  public boolean exampleCondition() {
    // Query some boolean state, such as a digital sensor.
    return false;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    intake.updateTelemetry();
  }

  @Override
  public void simulationPeriodic() {
    intake.simIterate();
  }
}
