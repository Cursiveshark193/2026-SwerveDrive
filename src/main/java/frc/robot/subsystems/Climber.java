// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Pounds;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import yams.gearing.GearBox;
import yams.gearing.MechanismGearing;
import yams.mechanisms.config.ElevatorConfig;
import yams.mechanisms.positional.Elevator;
import yams.motorcontrollers.SmartMotorController;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.SmartMotorControllerConfig.ControlMode;
import yams.motorcontrollers.SmartMotorControllerConfig.MotorMode;
import yams.motorcontrollers.SmartMotorControllerConfig.TelemetryVerbosity;

public class Climber extends SubsystemBase {

  private SmartMotorControllerConfig smcConfig = new SmartMotorControllerConfig(this)
    .withControlMode(ControlMode.OPEN_LOOP)
    // Mechanism Circumference is the distance traveled by each mechanism rotation converting rotations to meters.
    .withMechanismCircumference(Meters.of(Inches.of(0.25).in(Meters) * 22))
    // Feedback Constants (PID Constants)
    .withClosedLoopController(4, 0, 0, MetersPerSecond.of(0.5), MetersPerSecondPerSecond.of(0.5))
    .withSimClosedLoopController(4, 0, 0, MetersPerSecond.of(0.5), MetersPerSecondPerSecond.of(0.5))
    // Feedforward Constants
    .withFeedforward(new ElevatorFeedforward(0, 0, 0))
    .withSimFeedforward(new ElevatorFeedforward(0, 0, 0))
    // Telemetry name and verbosity level
    .withTelemetry("ElevatorMotor", TelemetryVerbosity.HIGH)
    // Gearing from the motor rotor to final shaft.
    // In this example GearBox.fromReductionStages(3,4) is the same as GearBox.fromStages("3:1","4:1") which corresponds to the gearbox attached to your motor.
    // You could also use .withGearing(12) which does the same thing.
    .withGearing(new MechanismGearing(GearBox.fromReductionStages(174.9)))
    // Motor properties to prevent over currenting.
    .withMotorInverted(false)
    .withIdleMode(MotorMode.BRAKE)
    .withStatorCurrentLimit(Amps.of(40))
    .withClosedLoopRampRate(Seconds.of(0.25))
    .withOpenLoopRampRate(Seconds.of(0.25));

  // Vendor motor controller object
  private VictorSPX spark = new VictorSPX(4); // example VictorSPX on CAN ID 4 (replace with your actual motor controller and ID)

  // Create our SmartMotorController from our Spark and config with a custom motor model.
  // We construct a realistic DCMotor using the provided motor constants (nominal 12V).
  // Constants provided: FreeSpeed=100 RPM, StallTorque=7.91 Nm, StallCurrent=24 A,
  // FreeCurrent (approx) = 1 A
  private SmartMotorController sparkSmartMotorController = new frc.robot.wrappers.VictorSPXWrapper(
    spark,
    new DCMotor(
      12.0, // nominal voltage (V)
      7.91, // stall torque (Nm)
      24.0, // stall current (A)
      1.0,  // free current (A) - approximated
      100.0 * 2.0 * Math.PI / 60.0, // free speed (rad/s) converted from 100 RPM
      1 // number of motors
    ),
    smcConfig);

  /*
   * Alternative B (explicit meters-per-radian mapping):
   * If you prefer to keep the default DCMotor but force the mechanism kinematics,
   * uncomment the lines below and comment out the constructor above.
   *
   * // double mechCirc = Meters.of(Inches.of(0.25).in(Meters) * 22);
   * // double rotorToMech = new MechanismGearing(GearBox.fromReductionStages(3,4)).getRotorToMechanismRatio();
   * // double metersPerRad = mechCirc / (2.0 * Math.PI * rotorToMech);
   * // private SmartMotorController sparkSmartMotorController = new frc.robot.wrappers.VictorSPXWrapper(spark, DCMotor.getNEO(1), smcConfig, metersPerRad);
   */

  private ElevatorConfig elevconfig = new ElevatorConfig(sparkSmartMotorController)
      .withStartingHeight(Meters.of(0.5))
      .withHardLimits(Meters.of(0), Meters.of(3))
      .withTelemetry("Elevator", TelemetryVerbosity.HIGH)
      .withMass(Pounds.of(16));

  // Elevator Mechanism
  private Elevator elevator = new Elevator(elevconfig);

  /**
   * Set the height of the elevator and does not end the command when reached.
   * @param angle Distance to go to.
   * @return a Command
   */
  public Command setHeight(Distance height) { return elevator.run(height);}
  
  /**
   * Set the height of the elevator and ends the command when reached, but not the closed loop controller.
   * @param angle Distance to go to.
   * @return A Command
   */
  public Command setHeightAndStop(Distance height) { return elevator.runTo(height, Meters.of(0));}
  
  /**
   * Set the elevators closed loop controller setpoint.
   * @param angle Distance to go to.
   */
  public void setHeightSetpoint(Distance height) { elevator.setMeasurementPositionSetpoint(height);}

  /**
   * Move the elevator up and down.
   * @param dutycycle [-1, 1] speed to set the elevator too.
   */
  public Command set(double dutycycle) { return elevator.set(dutycycle);}

  /**
   * Run sysId on the {@link Elevator}
   */
  public Command sysId() { return elevator.sysId(Volts.of(7), Volts.of(2).per(Second), Seconds.of(4));}

  /** Creates a new Climber subsystem. */
  public Climber() {}

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
   * An example method querying a boolean state of the subsystem (for example, a digital sensor).
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
    elevator.updateTelemetry();
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
    elevator.simIterate();
  }
}
