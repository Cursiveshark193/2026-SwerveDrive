package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.DegreesPerSecondPerSecond;
import static edu.wpi.first.units.Units.Feet;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Pounds;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.DutyCycle;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import yams.gearing.GearBox;
import yams.gearing.MechanismGearing;
import yams.mechanisms.config.ArmConfig;
import yams.mechanisms.positional.Arm;
import yams.motorcontrollers.SmartMotorController;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.SmartMotorControllerConfig.ControlMode;
import yams.motorcontrollers.SmartMotorControllerConfig.MotorMode;
import yams.motorcontrollers.SmartMotorControllerConfig.TelemetryVerbosity;
import yams.motorcontrollers.local.SparkWrapper;

public class arm extends SubsystemBase{
     private SmartMotorControllerConfig ArmConfig = new SmartMotorControllerConfig(this)
      .withControlMode(ControlMode.CLOSED_LOOP)
      // Feedback Constants (PID Constants)
      .withClosedLoopController(0, 0, 0, DegreesPerSecond.of(90), DegreesPerSecondPerSecond.of(45))
      .withSimClosedLoopController(0.025, 0, 0, DegreesPerSecond.of(20), DegreesPerSecondPerSecond.of(45))
      // Feedforward Constants
      .withFeedforward(new ArmFeedforward(1, 2, 0))
      .withSimFeedforward(new ArmFeedforward(1, 0, 0))
      // Telemetry name and verbosity level
      .withTelemetry("ArmMotor", TelemetryVerbosity.HIGH)
      // Gearing from the motor rotor to final shaft.
      // In this example GearBox.fromReductionStages(3,4) is the same as
      // GearBox.fromStages("3:1","4:1") which corresponds to the gearbox attached to
      // your motor.
      .withGearing(new MechanismGearing(GearBox.fromReductionStages(9)))
      // Motor properties to prevent over currenting.
      .withMotorInverted(true)
      .withIdleMode(MotorMode.COAST)
      .withStatorCurrentLimit(Amps.of(30))
      .withClosedLoopRampRate(Seconds.of(50))
      .withOpenLoopRampRate(Seconds.of(5));
  private SparkMax Arm = new SparkMax(16, MotorType.kBrushless); // create second SparkMax on CAN ID 17 (if needed)
  private SmartMotorController sparkSmartMotorController2 = new SparkWrapper(Arm, DCMotor.getNEO(1), ArmConfig);
  private final ArmConfig armCfg = new ArmConfig(sparkSmartMotorController2) // mechanism config for intake arm
      // Soft limit is applied to the SmartMotorControllers PID
      .withSoftLimits(Degrees.of(12.369033694267273), Degrees.of(-0.8214886207133532))
      // Hard limit is applied to the simulation.
      .withHardLimit(Degrees.of(12.369033694267273), Degrees.of(-4.631007201969624))
      // Starting position is where your arm starts
      .withStartingPosition(Degrees.of(-4.631007201969624))
      // Length and mass of your arm for sim.
      .withLength(Feet.of(1.75))
      .withMass(Pounds.of(3.45))
      // Telemetry name and verbosity for the arm.
      .withTelemetry("Army", TelemetryVerbosity.HIGH);
  private Arm arm = new Arm(armCfg); // create Arm mechanism instance with arm config

 public Command Set_To_90_Degrees() {
    return arm.setAngle(Degrees.of(90)); // example command to set arm to 90 degrees (replace with desired angle)
  }
  public Command StowArm () {
    return arm.setAngle(Degrees.of(-4.631007201969624)); // example command to stow arm at starting position (replace with desired angle)
  } 
  public Command runAtSpeed () {
    return run(() -> arm.setDutyCycleSetpoint(0.3)); // example command to run arm at given speed (replace with desired speed)
  } 
  public Command Agitate () {
    return arm.setAngle(Degrees.of(10)).withTimeout(Seconds.of(0.5)).andThen(arm.setAngle(Degrees.of(-4.631007201969624)).withTimeout(Seconds.of(0.5))); // example command to agitate arm by moving to 10 degrees and back to starting position (replace with desired angles and timings)
  }
  public Command OnStandby () {
    return arm.setAngle(Degrees.of(0)); // example command to hold arm at 0 degrees (replace with desired angle)
  }
 /** Create a new Intake subsystem. */
  public arm() {
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
    arm.updateTelemetry();
  }

  @Override
  public void simulationPeriodic() {
    arm.simIterate();
  }
}