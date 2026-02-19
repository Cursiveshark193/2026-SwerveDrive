package frc.robot.subsystems; // package for robot subsystem classes

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

/**
 * ShooterSubsystem controls the shooter motors and their simulated mechanisms.
 *
 * <p>This subsystem exposes commands to set open-loop duty or closed-loop velocity,
 * run a short follower direction test, and perform a simple system identification
 * routine. It keeps both a leader SmartMotorController and a separate follower
 * SmartMotorController wrapper so that both devices are visible in telemetry and
 * the simulator.
 */
public class ShooterSubsystem extends SubsystemBase { // subsystem that encapsulates shooter hardware and simulation

  private final TalonFX ShooterMotorLeader = new TalonFX
                                              (
                                              14, 
                                              "rio"
                                              ); 
                                              // instantiate leader TalonFX on 
                                             //CAN ID 14 on "rio" bus

   // optional telemetry config (commented out): controls which additional
  // telemetry fields are published for a SmartMotorController.
  private final SmartMotorControllerConfig motorConfig = new SmartMotorControllerConfig(this)// configuration object
                                                                                            // for the leader wrapper
                                                                                        .withFollowers(Pair.of(
                                                                                          new TalonFX(
                                                                                            15, 
                                                                                            "rio"), 
                                                                                            true)) 
                                                                                            // register a hardware follower 
                                                                                           //(TalonFX ID 15) 
                                                                                          //and mark it inverted
                                                                                        .withClosedLoopController(
                                                                                          0.00016541, 
                                                                                          0, 
                                                                                          0, 
                                                                                          RPM
                                                                                          .of(5000),
                                                                                          RotationsPerSecondPerSecond
                                                                                          .of(2500)) 
                                                                                          // set PID
                                                                                         // constants and
                                                                                        // closed-loop
                                                                                       // safety limits
                                                                                          .withSimClosedLoopController(
                                                                                            0,
                                                                                            0, 
                                                                                            0, 
                                                                                            RPM
                                                                                            .of(5000), 
                                                                                            RotationsPerSecondPerSecond
                                                                                            .of(2500)) 
                                                                                            // set simulated
                                                                                           // closed-loop defaults
                                                                                          .withGearing(
                                                                                            new MechanismGearing(
                                                                                              GearBox.
                                                                                              fromReductionStages
                                                                                              (3, 4))) 
                                                                                          // configure gearing between motor and
                                                                                         // mechanism
                                                                                         .withIdleMode(
                                                                                          MotorMode.COAST) 
                                                                                         // set idle mode to coast
                                                                                         .withTelemetry(
                                                                                          "ShooterMotor", 
                                                                                          TelemetryVerbosity.HIGH) 
                                                                                          // telemetry name and verbosity level
                                                                                         .withStatorCurrentLimit(
                                                                                          Amps.of
                                                                                          (40)) 
                                                                                         // limit stator current to protect hardware
                                                                                         .withMotorInverted(
                                                                                          false) 
                                                                                          // do not invert leader motor output
                                                                                         .withClosedLoopRampRate(
                                                                                          Seconds.of
                                                                                          (25)) 
                                                                                          // apply closed-loop ramping
                                                                                         .withOpenLoopRampRate(
                                                                                          Seconds.of
                                                                                          (25)) 
                                                                                          // apply open-loop ramping
                                                                                         .withFeedforward(
                                                                                          new SimpleMotorFeedforward
                                                                                          (0.27937, 
                                                                                           0.089836,
                                                                                           0.014557)) 
                                                                                           // feedforward gains used in control
                                                                                         .withSimFeedforward(
                                                                                          new SimpleMotorFeedforward
                                                                                          (0.27937, 
                                                                                           0.089836, 
                                                                                           0.014557)) 
                                                                                          // same feedforward for simulation
                                                                                         .withControlMode(
                                                                                          ControlMode.CLOSED_LOOP); // use closed-loop control mode by default
  private final SmartMotorController motor = new TalonFXWrapper(ShooterMotorLeader, 
                                                                DCMotor
                                                                .getKrakenX60(1),
                                                                motorConfig); 
  // leader wrapper adapts TalonFX to SmartMotorController

 

  private final FlyWheelConfig shooterConfig = new FlyWheelConfig(motor) // mechanism model configuration for leader
                                                                 .withDiameter(Inches.of(6)) // flywheel diameter
                                                                 .withMass(Pounds.of(10)) // flywheel mass
                                                                 .withTelemetry(
                                                                  "ShooterMech", 
                                                                  TelemetryVerbosity.HIGH) // mechanism telemetry name
                                                                 .withSoftLimit(RPM.of(
                                                                  -50000), 
                                                                  RPM.of(
                                                         50000)); // soft limits for speed
  // .withSpeedometerSimulation(RPM.of(0)); // optional: initial simulated speed
  private final FlyWheel shooter = new FlyWheel(shooterConfig);
                         // create FlyWheel mechanism for leader 





  private final TalonFX ShooterMotorFollower = new TalonFX(
                                                  15, 
                                                  "rio"
                                                  ); 
                                                  // instantiate follower TalonFX on 
                                                 //CAN ID 15 on 'rio' bus
                                                                       

  // followerConfig: create a separate wrapper so follower shows in telemetry/sim
  private final SmartMotorControllerConfig followerConfig = new SmartMotorControllerConfig(this)
                                                                .withIdleMode(
                                                                  MotorMode.COAST) 
                                                                  // follower idle mode
                                                                .withTelemetry(
                                                                "ShooterFollower", 
                                                                TelemetryVerbosity.HIGH) 
                                                                // follower telemetry name
                                                                .withGearing(new MechanismGearing
                                                                (GearBox
                                                                .fromReductionStages
                                                                (3, 4))) 
                                                                // mirror gearing
                                                                .withClosedLoopController(
                                                                  0.00016541, 
                                                                  0, 
                                                                  0, 
                                                                  RPM.of
                                                                  (5000), 
                                                                  RotationsPerSecondPerSecond.of
                                                                  (2500)) 
                                                                  // mirror PID
                                                                .withSimClosedLoopController(
                                                                  0, 
                                                                  0, 
                                                                  0, 
                                                                  RPM.of
                                                                  (5000), 
                                                                  RotationsPerSecondPerSecond.of
                                                                  (2500)) 
                                                                  // mirror sim PID
                                                                .withStatorCurrentLimit(
                                                                  Amps.of
                                                                  (40)) 
                                                                  // mirror current limit
                                                                .withMotorInverted(
                                                                  true) 
                                                                  // match physical follower inversion flag
                                                                .withClosedLoopRampRate(
                                                                  Seconds.of
                                                                  (25)) 
                                                                  // mirror ramp rates
                                                                .withOpenLoopRampRate(
                                                                  Seconds.of
                                                                  (25))
                                                                .withControlMode(
                                                                  ControlMode.
                                                                  CLOSED_LOOP
                                                                  ); 
                                                                  // same control mode
  
private final SmartMotorController follower = new TalonFXWrapper(
                                                   ShooterMotorFollower, 
                                                   DCMotor.
                                                    getKrakenX60(1),
                                                   followerConfig); 
                                                   // follower wrapper
  private final FlyWheelConfig shooterfollowConfig = new FlyWheelConfig(follower) 
                                                         // mechanism model for
                                                        // follower wrapper
                                                         .withDiameter(
                                                          Inches.of
                                                          (6)) 
                                                          // match diameter
                                                         .withMass(
                                                          Pounds.of
                                                          (10)) 
                                                         // match mass
                                                         .withTelemetry(
                                                          "Shooter", 
                                                          TelemetryVerbosity.HIGH) 
                                                          // telemetry name for follower mechanism
                                                         .withSoftLimit(
                                                          RPM.of
                                                          (-50000), 
                                                          RPM.of
                                                 (50000)); 
                                                 // same soft limits
  // .withSpeedometerSimulation(RPM.of(0));
  private final FlyWheel shooterfollow = new FlyWheel(shooterfollowConfig); // create FlyWheel for follower
  /**
   * Create a new ShooterSubsystem.
   */
  public ShooterSubsystem() { // default constructor; no extra initialization needed
  }

  /**
   * Get the current shooter wheel angular velocity.
   *
   * @return current angular velocity of the leader shooter mechanism
   */
  public AngularVelocity getVelocity() { // return current mechanism angular velocity
    return shooter.getSpeed();
  }

  /**
   * Convenience command: spin both leader and follower to ~3000 RPM while
   * the command runs. This composes the leader command with the follower
   * command so both appear in telemetry and simulation.
   */
  public Command SpinAt3000RPM() {
    return shooter.setSpeed(RPM.of(3000))
        .andThen(shooterfollow.setSpeed(RPM.of(3000)));
  }

  /**
   * Close-loop to a fixed angular velocity (one-shot command).
   *
   * @param velocity the target angular velocity
   * @return a command that drives the leader to the requested velocity
   */
  public Command setVelocity(AngularVelocity velocity) {
    return shooter.setSpeed(velocity);
  }

  /**
   * Stop the shooter (set velocity to zero).
   *
   * @return a command that stops the shooter
   */
  public Command Stop() {
    return shooter.setSpeed(RPM.of(0));
  }

  /**
   * Create a command to set an open-loop duty cycle on the leader shooter
   * motor.
   *
   * @param dutyCycle duty cycle in the range [-1.0, 1.0]
   * @return a {@link Command} that when scheduled will apply the requested
   *     duty cycle
   */
  public Command setDutyCycle(double dutyCycle) { // create a command to set open-loop duty on the leader
    return shooter.set(dutyCycle);
  }

  /**
   * Run a short test to verify follower direction.
   *
   * <p>This schedules an open-loop duty on the leader for a fixed duration so
   * you can observe the physical follower or the simulated follower and confirm
   * its inversion setting.
   *
   * @param duty duty cycle to apply (e.g. 0.2 for 20%)
   * @param seconds duration in seconds to run the test
   * @return a {@link Command} that runs the shooter for the given duration
   */
  public Command runFollowerTest(double duty, double seconds) { // helper command to run leader briefly for verification
    return setDutyCycle(duty).withTimeout(seconds); // run leader at duty for duration; observe follower
  }

  /**
   * Create a command that closes the loop to a velocity supplied at runtime.
   *
   * @param speed supplier that provides the target angular velocity
   * @return a {@link Command} that when scheduled will drive the shooter to the
   *     supplied speed
   */
  public Command setVelocity(Supplier<AngularVelocity> speed) { // create a command that sets velocity from a supplier
    return shooter.setSpeed(speed);
  }

  /**
   * Create a command that applies an open-loop duty cycle supplied at runtime.
   *
   * @param dutyCycle supplier that provides duty in the range [-1.0, 1.0]
   * @return a {@link Command} that when scheduled will apply the supplied duty
   *     cycle to the leader
   */
  public Command setDutyCycle(Supplier<Double> dutyCycle) { // command to set duty from a supplier
    return shooter.set(dutyCycle);
  }

  /**
   * Create a simple system-identification command for the shooter mechanism.
   *
   * <p>This runs a brief voltage-step protocol used for feedforward/tuning
   * measurements.
   *
   * @return a {@link Command} that performs the sysid routine
   */
  public Command sysId() { // system identification helper command
    return shooter.sysId(Volts.of(10), Volts.of(1).per(Second), Seconds.of(5));
  }

  @Override
  public void periodic() { // regular scheduler-periodic updates
    shooter.updateTelemetry(); // update leader telemetry fields
    follower.updateTelemetry(); // update follower telemetry fields to keep it visible in dashboards
  }

  @Override
  public void simulationPeriodic() { // simulation-only updates called at sim rate
    shooter.simIterate(); // update leader simulation
    shooterfollow.simIterate(); // update follower simulation to keep it in sync with leader
  }
}