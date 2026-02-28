package frc.robot.wrappers;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.AngularAccelerationUnit;
import edu.wpi.first.units.measure.Velocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Volts;
// Radians not used directly after switching to gearing-based conversions
import yams.motorcontrollers.SimSupplier;
import yams.motorcontrollers.SmartMotorController;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.SmartMotorFactory;

import java.util.Optional;
import java.util.OptionalDouble;

/**
 * Small adapter that exposes a VictorSPX as a YAMS SmartMotorController.
 * This implementation is intentionally minimal: it provides the basic control
 * and telemetry hooks and stubs out other features so the project compiles.
 * If you need advanced features (closed-loop, PID tuning, sim encoder seeding)
 * expand the implementations below or copy patterns from the YAMS SparkWrapper.
 */
public class VictorSPXWrapper extends SmartMotorController {

  static {
    // Best-effort registration with YAMS SmartMotorFactory so callers can
    // call SmartMotorFactory.create(vendor, ...) and get this wrapper.
    try {
      java.lang.reflect.Method register = SmartMotorFactory.class.getDeclaredMethod("register", String.class, String.class);
      register.setAccessible(true);
      register.invoke(null, "com.ctre.phoenix.motorcontrol.can.VictorSPX", VictorSPXWrapper.class.getName());
    } catch (Throwable t) {
      // ignore if registration isn't possible or already registered
    }
  }

  private final VictorSPX ctrl;
  private final DCMotor motorModel;
  private final SmartMotorControllerConfig config;

  // Track last commanded duty for telemetry / simulation
  private volatile double lastDuty = 0.0;

  // Simple simulated state (keeps wrapper useful for sim without full CTRE sim bindings)
  private double rotorPositionRad = 0.0;
  private double rotorVelocityRadPerSec = 0.0;
  private double mechanismPositionMeters = 0.0;
  private double mechanismVelocityMps = 0.0;
  // Conversion helpers derived from SmartMotorControllerConfig
  private final double rotorToMechanismRatio;
  private final double metersPerRad; // meters of mechanism travel per 1 rad of rotor

  /**
   * Create a VictorSPXWrapper using the provided YAMS config. The wrapper will
   * derive the rotor->mechanism conversion from the config if available.
   */
  public VictorSPXWrapper(VictorSPX controller, DCMotor motorModel, SmartMotorControllerConfig config) {
    this(controller, motorModel, config, Double.NaN);
  }

  /**
   * Create a VictorSPXWrapper and explicitly provide meters-per-radian mapping.
   * If {@code overrideMetersPerRad} is NaN the constructor will derive the
   * conversion from {@link SmartMotorControllerConfig} as before.
   *
   * @param overrideMetersPerRad meters of mechanism travel per 1 rad of rotor
   */
  public VictorSPXWrapper(VictorSPX controller, DCMotor motorModel, SmartMotorControllerConfig config, double overrideMetersPerRad) {
    this.ctrl = controller;
    this.motorModel = motorModel;
    this.config = config;

    // Basic initialization; adapt to your CTRE API version
    try {
      ctrl.configFactoryDefault();
    } catch (Exception e) {
      // older/newer CTRE APIs might not throw; ignore if not supported
    }

    // Apply configured inversion if available on config
    try {
      ctrl.setInverted(config.getMotorInverted()); // some APIs use setInverted(boolean)
    } catch (NoSuchMethodError | Exception ex) {
      // If your CTRE version uses a different call, adapt here
    }

    // Compute gearing-derived conversion. If override provided use it, otherwise
    // derive from SmartMotorControllerConfig (mechanism circumference & gearing).
    double mechCircMeters = 0.01; // fallback: small default
    try {
      mechCircMeters = config.getMechanismCircumference().map(d -> d.in(Meters)).orElse(mechCircMeters);
    } catch (Throwable t) {
      // ignore and use default
    }
    double r2m = 1.0;
    try {
      r2m = config.getGearing().getRotorToMechanismRatio();
    } catch (Throwable t) {
      r2m = 1.0;
    }
    this.rotorToMechanismRatio = r2m;

    if (!Double.isNaN(overrideMetersPerRad)) {
      this.metersPerRad = overrideMetersPerRad;
    } else {
      this.metersPerRad = mechCircMeters / (2.0 * Math.PI * rotorToMechanismRatio);
    }
    // Ensure the YAMS base class sees the configured SmartMotorControllerConfig
    // Many YAMS base-class methods (including setupTelemetry) expect m_config
    // to be non-null during mechanism construction. Assign the protected
    // field here so telemetry/setup code doesn't NPE during simulation startup.
    this.m_config = config;
  }

  // --- Basic control API that YAMS expects --- //

  @Override
  public void setDutyCycle(double duty) {
    lastDuty = duty;
    // Send to hardware. Use PercentOutput control mode.
    ctrl.set(ControlMode.PercentOutput, duty);
  }

  @Override
  public double getDutyCycle() {
    // If CTRE provides getMotorOutputPercent(), use it; else return our last duty
    try {
      return ctrl.getMotorOutputPercent();
    } catch (Throwable t) {
      return lastDuty;
    }
  }

  @Override
  public void setMotorInverted(boolean inverted) {
    try {
      ctrl.setInverted(inverted);
    } catch (Throwable t) {
      // ignore or log
    }
  }

  @Override
  public SmartMotorControllerConfig getConfig() {
    return config;
  }

  @Override
  public void updateTelemetry() {
    // Publish a concise, well-named set of telemetry fields under the
    // configured telemetry namespace. Keep allocations and work minimal.
  final String base = config.getTelemetryName().orElse("VictorSPX");
    double duty = getDutyCycle();
    SmartDashboard.putNumber(base + "/duty", duty);
    // Rotor (deg) and rotor velocity (deg/s)
    try {
      SmartDashboard.putNumber(base + "/rotorPositionDeg", Math.toDegrees(rotorPositionRad));
      SmartDashboard.putNumber(base + "/rotorVelocityDegPerSec", Math.toDegrees(rotorVelocityRadPerSec));
    } catch (Throwable t) {
      // defensive: if unit conversion isn't available, skip
    }
    // Mechanism linear position (m) and velocity (m/s)
    SmartDashboard.putNumber(base + "/mechPositionM", mechanismPositionMeters);
    SmartDashboard.putNumber(base + "/mechVelocityMps", mechanismVelocityMps);
    // Voltage and stator current approximations from the wrapper
    try {
      SmartDashboard.putNumber(base + "/voltageV", getVoltage().in(Volts));
    } catch (Throwable t) {
      SmartDashboard.putNumber(base + "/voltageV", duty * 12.0);
    }
    try {
      SmartDashboard.putNumber(base + "/statorCurrentA", getStatorCurrent().in(Amps));
    } catch (Throwable t) {
      SmartDashboard.putNumber(base + "/statorCurrentA", Math.abs(duty) * 10.0);
    }
  }

  @Override
  public void simIterate() {
    // Very small, safe sim model so telemetry shows movement in Simulation.
    // This is intentionally simple: it drives rotor velocity proportional to duty
    // and integrates to a rotor position. The mechanism position is derived
    // by a small conversion factor so mechanisms (like elevators) show motion.
    double dt = 0.02; // 20ms sim step (matches WPILib default)
    double maxRadPerSec = 50.0; // rough max angular speed for simulation
    rotorVelocityRadPerSec = lastDuty * maxRadPerSec;
    rotorPositionRad += rotorVelocityRadPerSec * dt;

    // Convert rotor motion to mechanism linear travel using configured gearing/circumference
    mechanismPositionMeters = rotorPositionRad * metersPerRad;
    mechanismVelocityMps = rotorVelocityRadPerSec * metersPerRad;
  }

  @Override
  public void setupSimulation() {
    // Create a lightweight SimSupplier that bridges YAMS sim requests to
    // this wrapper's simple sim state. This allows YAMS to seed and read
    // mechanism/rotor positions and voltages.
    SimSupplier supplier = new SimSupplier() {
      private volatile boolean updated = false;
      private volatile boolean inputFed = true;

      @Override
      public void updateSimState() {
        // Called by YAMS periodically to let the supplier advance state.
        // We'll reuse the same integration as simIterate for consistency.
        simIterate();
        updated = true;
      }

      @Override
      public boolean getUpdatedSim() { return updated; }

      @Override
      public void feedUpdateSim() { updated = true; }

      @Override
      public void starveUpdateSim() { updated = false; }

      @Override
      public boolean isInputFed() { return inputFed; }

      @Override
      public void feedInput() { inputFed = true; }

      @Override
      public void starveInput() { inputFed = false; }

      @Override
      public void setMechanismStatorDutyCycle(double duty) { lastDuty = duty; }

      @Override
      public edu.wpi.first.units.measure.Voltage getMechanismSupplyVoltage() { return Volts.of(lastDuty * 12.0); }

      @Override
      public edu.wpi.first.units.measure.Voltage getMechanismStatorVoltage() { return Volts.of(lastDuty * 12.0); }

      @Override
      public void setMechanismStatorVoltage(edu.wpi.first.units.measure.Voltage v) { try { setVoltage(v); } catch (Throwable t) {} }

      @Override
      public edu.wpi.first.units.measure.Angle getMechanismPosition() {
        // Mechanism angle (deg) = rotor angle (deg) / rotorToMechanismRatio
        double mechDeg = Math.toDegrees(rotorPositionRad) / rotorToMechanismRatio;
        return Degrees.of(mechDeg);
      }

      @Override
      public void setMechanismPosition(edu.wpi.first.units.measure.Angle a) {
        // Given a mechanism angle, compute rotor position and update linear metric
        double mechDeg = a.in(Degrees);
        double rotorDeg = mechDeg * rotorToMechanismRatio;
        rotorPositionRad = Math.toRadians(rotorDeg);
        mechanismPositionMeters = rotorPositionRad * metersPerRad;
      }

      @Override
      public edu.wpi.first.units.measure.Angle getRotorPosition() { return Degrees.of(Math.toDegrees(rotorPositionRad)); }

      @Override
      public edu.wpi.first.units.measure.AngularVelocity getMechanismVelocity() {
        // Mechanism angular velocity = rotor angular velocity / rotorToMechanismRatio
        double mechDegPerSec = Math.toDegrees(rotorVelocityRadPerSec) / rotorToMechanismRatio;
        return DegreesPerSecond.of(mechDegPerSec);
      }

      @Override
      public void setMechanismVelocity(edu.wpi.first.units.measure.AngularVelocity v) {
        double mechDegPerSec = v.in(DegreesPerSecond);
        double rotorDegPerSec = mechDegPerSec * rotorToMechanismRatio;
        rotorVelocityRadPerSec = Math.toRadians(rotorDegPerSec);
        mechanismVelocityMps = rotorVelocityRadPerSec * metersPerRad;
      }

      @Override
      public edu.wpi.first.units.measure.AngularVelocity getRotorVelocity() { return DegreesPerSecond.of(Math.toDegrees(rotorVelocityRadPerSec)); }

      @Override
      public edu.wpi.first.units.measure.Current getCurrentDraw() { return Amps.of(Math.abs(lastDuty) * 10.0); }
    };

    // Register supplier with the base class so the rest of YAMS can use it.
    try {
      setSimSupplier(supplier);
    } catch (Throwable t) {
      // Ignore if base class doesn't accept the supplier for any reason.
    }
  }

  // --- Minimal stubs for the abstract surface on SmartMotorController ---
  // The goal here is to provide safe, compile-time implementations. If your
  // mechanisms rely on any of these, replace the stub with a real implementation.

  

  @Override
  public void seedRelativeEncoder() {}

  @Override
  public void synchronizeRelativeEncoder() {}

  @Override
  public void setIdleMode(SmartMotorControllerConfig.MotorMode mode) {}

  @Override
  public void setEncoderVelocity(edu.wpi.first.units.measure.AngularVelocity v) {}

  @Override
  public void setEncoderVelocity(edu.wpi.first.units.measure.LinearVelocity v) {}

  @Override
  public void setEncoderPosition(edu.wpi.first.units.measure.Angle a) {}

  @Override
  public void setEncoderPosition(edu.wpi.first.units.measure.Distance d) {}

  @Override
  public void setPosition(edu.wpi.first.units.measure.Angle a) {}

  @Override
  public void setPosition(edu.wpi.first.units.measure.Distance d) {}

  @Override
  public void setVelocity(edu.wpi.first.units.measure.LinearVelocity v) {}

  @Override
  public void setVelocity(edu.wpi.first.units.measure.AngularVelocity v) {}

  @Override
  public boolean applyConfig(SmartMotorControllerConfig cfg) {
    // We already applied what we needed in the ctor; return true to indicate success.
    return true;
  }

  @Override
  public java.util.Optional<edu.wpi.first.units.measure.Current> getSupplyCurrent() {
    return Optional.empty();
  }

  @Override
  public edu.wpi.first.units.measure.Current getStatorCurrent() { return Amps.of(Math.abs(lastDuty) * 10.0); }

  @Override
  public edu.wpi.first.units.measure.Voltage getVoltage() { return Volts.of(lastDuty * 12.0); }

  @Override
  public void setVoltage(edu.wpi.first.units.measure.Voltage voltage) { 
    try {
      double v = voltage.in(Volts);
      lastDuty = Math.max(-1.0, Math.min(1.0, v / 12.0));
    } catch (Throwable t) {
      // ignore if conversion not supported
    }
  }

  @Override
  public DCMotor getDCMotor() { return motorModel; }

  @Override
  public edu.wpi.first.units.measure.LinearVelocity getMeasurementVelocity() { return MetersPerSecond.of(mechanismVelocityMps); }

  @Override
  public edu.wpi.first.units.measure.Distance getMeasurementPosition() {
    try {
      // prefer YAMS conversion utilities when available
      return config.convertFromMechanism(getMechanismPosition());
    } catch (Throwable t) {
      return Meters.of(mechanismPositionMeters);
    }
  }

  @Override
  public edu.wpi.first.units.measure.AngularVelocity getMechanismVelocity() {
    double mechDegPerSec = Math.toDegrees(rotorVelocityRadPerSec) / rotorToMechanismRatio;
    return DegreesPerSecond.of(mechDegPerSec);
  }

  @Override
  public edu.wpi.first.units.measure.Angle getMechanismPosition() {
    double mechDeg = Math.toDegrees(rotorPositionRad) / rotorToMechanismRatio;
    return Degrees.of(mechDeg);
  }

  @Override
  public edu.wpi.first.units.measure.AngularVelocity getRotorVelocity() { return DegreesPerSecond.of(Math.toDegrees(rotorVelocityRadPerSec)); }

  @Override
  public edu.wpi.first.units.measure.Angle getRotorPosition() { return Degrees.of(Math.toDegrees(rotorPositionRad)); }

  @Override
  public void setEncoderInverted(boolean inverted) {}

  @Override
  public void setMotionProfileMaxVelocity(edu.wpi.first.units.measure.LinearVelocity v) {}

  @Override
  public void setMotionProfileMaxAcceleration(edu.wpi.first.units.measure.LinearAcceleration a) {}

  @Override
  public void setMotionProfileMaxVelocity(edu.wpi.first.units.measure.AngularVelocity v) {}

  @Override
  public void setMotionProfileMaxAcceleration(edu.wpi.first.units.measure.AngularAcceleration a) {}

  @Override
  public void setKp(double kp) {}

  @Override
  public void setKi(double ki) {}

  @Override
  public void setKd(double kd) {}

  @Override
  public void setFeedback(double a, double b, double c) {}

  @Override
  public void setKs(double ks) {}

  @Override
  public void setKv(double kv) {}

  @Override
  public void setKa(double ka) {}

  @Override
  public void setKg(double kg) {}

  @Override
  public void setFeedforward(double a, double b, double c, double d) {}

  @Override
  public void setStatorCurrentLimit(edu.wpi.first.units.measure.Current c) {}

  @Override
  public void setSupplyCurrentLimit(edu.wpi.first.units.measure.Current c) {}

  @Override
  public void setClosedLoopRampRate(edu.wpi.first.units.measure.Time t) {}

  @Override
  public void setOpenLoopRampRate(edu.wpi.first.units.measure.Time t) {}

  @Override
  public void setMeasurementUpperLimit(edu.wpi.first.units.measure.Distance d) {}

  @Override
  public void setMeasurementLowerLimit(edu.wpi.first.units.measure.Distance d) {}

  @Override
  public void setMechanismUpperLimit(edu.wpi.first.units.measure.Angle a) {}

  @Override
  public void setMechanismLowerLimit(edu.wpi.first.units.measure.Angle a) {}

  @Override
  public edu.wpi.first.units.measure.Temperature getTemperature() { return null; }

  @Override
  public java.lang.Object getMotorController() { return ctrl; }

  @Override
  public java.lang.Object getMotorControllerConfig() { return config; }

  @Override
  public Pair<java.util.Optional<java.util.List<yams.telemetry.SmartMotorControllerTelemetry.BooleanTelemetryField>>, java.util.Optional<java.util.List<yams.telemetry.SmartMotorControllerTelemetry.DoubleTelemetryField>>> getUnsupportedTelemetryFields() {
    return new Pair<>(Optional.empty(), Optional.empty());
  }

  @Override
  public void setMotionProfileMaxJerk(Velocity<AngularAccelerationUnit> maxJerk) {
    // TODO Auto-generated method stub
    throw new UnsupportedOperationException("Unimplemented method 'setMotionProfileMaxJerk'");
  }

  @Override
  public void setExponentialProfile(OptionalDouble kV, OptionalDouble kA, Optional<Voltage> maxInput) {
    // TODO Auto-generated method stub
    throw new UnsupportedOperationException("Unimplemented method 'setExponentialProfile'");
  }
}