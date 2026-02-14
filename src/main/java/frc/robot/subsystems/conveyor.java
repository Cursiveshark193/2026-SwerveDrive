package frc.robot.subsystems;


import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.DegreesPerSecondPerSecond;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Pounds;
import static edu.wpi.first.units.Units.RPM;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
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
public class conveyor extends SubsystemBase {
    private SmartMotorControllerConfig ConveyorConfig = new SmartMotorControllerConfig(this) // separate config for conveyor if needed (can also reuse smcConfig if settings are the same)
      .withControlMode(ControlMode.CLOSED_LOOP) // example open-loop control mode
      .withClosedLoopController(0, 0, 0, DegreesPerSecond.of(30), DegreesPerSecondPerSecond.of(45)) // example PID constants +
                                                                                                    // safety
                                                                                                    // velocity/accel
                                                                                                    // limits
      .withSimClosedLoopController(0, 0, 0, DegreesPerSecond.of(30), DegreesPerSecondPerSecond.of(45)) // sim PID defaults
      .withFeedforward(new SimpleMotorFeedforward(0, 0, 0)) // example feedforward gains
      .withSimFeedforward(new SimpleMotorFeedforward(0, 0, 0)) // sim feedforward gains
      .withTelemetry("conveyorMotor", TelemetryVerbosity.HIGH) // telemetry label for conveyor motor
      .withGearing(new MechanismGearing(GearBox.fromReductionStages(5))) // mechanism gearing config
      .withMotorInverted(false) // motor inversion setting
      .withIdleMode(MotorMode.COAST) // idle/coast behavior
      .withStatorCurrentLimit(Amps.of(40)); // stator current limit to protect hardware

  private SparkMax conveyorSpark = new SparkMax(17, MotorType.kBrushless); // example second SparkMax for conveyor on CAN ID 17
  // Create SmartMotorController for conveyor SparkMax with its config and motor model (NEO550)
  private SmartMotorController conveyorSmartMotorController = new SparkWrapper(conveyorSpark, DCMotor.getNeo550(1), ConveyorConfig); // wrap second SparkMax with SmartMotorController for conveyor

  private final FlyWheelConfig conveyorConfig = new FlyWheelConfig(conveyorSmartMotorController) // mechanism config for conveyor
      .withDiameter(Inches.of(2)) // example diameter for conveyor roller
      .withMass(Pounds.of(5)) // example mass for conveyor roller
      .withUpperSoftLimit(RPM.of(100)) // example upper soft speed limit for conveyor
      .withTelemetry("conveyorMech", TelemetryVerbosity.HIGH); // telemetry label for conveyor mechanism

public FlyWheel conveyor = new FlyWheel(conveyorConfig); // example second mechanism for conveyor)

  public Command RunConveyor () {
    return conveyor.setSpeed(RPM.of(100)); // example command to set flywheel to 100 RPM (replace with desired speed)
  }
 public Command StopConveyor() {
   return conveyor.setSpeed(RPM.of(0)); // command to stop conveyor
 }
 public Command ReverseConveyor() {
   return conveyor.set(-0.3); // example command to run conveyor in reverse at 30% (replace with desired speed)
 }
 
public conveyor() {
    
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
    conveyor.updateTelemetry();  }

  @Override
  public void simulationPeriodic() {
    conveyor.simIterate();
  }
}


    // constructor can be used for additional setup if needed  
  



