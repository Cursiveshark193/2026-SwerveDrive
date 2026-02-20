 package frc.robot.subsystems; // package for robot subsystem classes

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Pounds;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;
import static edu.wpi.first.units.Units.Seconds;

import com.revrobotics.spark.SparkMax;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import yams.gearing.GearBox;
import yams.gearing.MechanismGearing;
import yams.mechanisms.config.FlyWheelConfig;
import yams.mechanisms.velocity.FlyWheel;
import yams.motorcontrollers.SmartMotorController;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.SmartMotorControllerConfig.ControlMode;
import yams.motorcontrollers.SmartMotorControllerConfig.MotorMode;
import yams.motorcontrollers.SmartMotorControllerConfig.TelemetryVerbosity;
import yams.motorcontrollers.local.SparkWrapper;

public class FeederSubsystem extends SubsystemBase {
 private  SparkMax ShooterFeeder = new SparkMax(
                                              19, 
                                              com.revrobotics.spark.SparkLowLevel.MotorType.kBrushless
                                              ); 
                                              // example additional motor (e.g. for a feeder) on CAN ID 16; not wrapped with SmartMotorController so it won't appear in YAMS telemetry/simulation but can still be controlled with SparkMax methods
  private final SmartMotorControllerConfig ShooterFeederConfig = new SmartMotorControllerConfig(this)
                                                                .withFollowers(Pair.of(
                                                                   new SparkMax(20,
                                                                com.revrobotics.spark.SparkLowLevel.MotorType.kBrushless),
                                                                true)) 
                                                                // register a hardware follower
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
                                                                (5))) 
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
  private final SmartMotorController ShooterFeederController = new SparkWrapper(
                                                                    ShooterFeeder, 
                                                                    DCMotor.getNEO
                                                                    (1), 
                                                                    ShooterFeederConfig); 
                                                                    // wrap feeder SparkMax with SmartMotorController to include in YAMS telemetry/simulation
  private final FlyWheelConfig ShooterFeederMechConfig = new FlyWheelConfig(ShooterFeederController) 
                                                         // mechanism model for
                                                        // feeder motor wrapper
                                                         .withDiameter(
                                                          Inches.of
                                                          (3)) 
                                                          // example diameter for feeder wheel
                                                         .withMass(
                                                          Pounds.of
                                                          (5)) 
                                                         // example mass for feeder wheel
                                                         .withTelemetry(
                                                          "ShooterFeeder", 
                                                          TelemetryVerbosity.HIGH) 
                                                          // telemetry name for feeder mechanism
                                                         .withSoftLimit(
                                                          RPM.of
                                                          (-50000), 
                                                          RPM.of
                                                 (50000)); 
                                                 // same soft limits  
  private final FlyWheel ShooterFeederMech = new FlyWheel(ShooterFeederMechConfig); // create FlyWheel for feeder mechanism 
  public FeederSubsystem() {
    // constructor can be used for additional setup if needed
}  
  public AngularVelocity getFeederVelocity() { // return current mechanism angular velocity
    return ShooterFeederMech.getSpeed();
  }
  public Command SpinAt1000RPM() {
    return ShooterFeederMech.setSpeed(RPM.of(1000));
  }
  public Command Reverse () {
    return ShooterFeederMech.setSpeed(RPM.of(-1000)); // example command to run feeder in reverse at 1000 RPM (replace with desired speed)
  }
  public Command Stop() {
  return ShooterFeederMech.setSpeed(RPM.of(0)); // command to stop feeder by setting speed to 0 RPM
} 
@Override
  public void periodic() { // regular scheduler-periodic updates
    ShooterFeederMech.updateTelemetry(); // update leader telemetry fields
    //follower.updateTelemetry(); // update follower telemetry fields to keep it visible in dashboards
  }

  @Override
  public void simulationPeriodic() { // simulation-only updates called at sim rate
    ShooterFeederMech.simIterate(); // update leader simulation
    //shooterfollow.simIterate();
  }
}
                                            