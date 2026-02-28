/*package frc.robot.commands;

import java.util.function.BooleanSupplier;

import static edu.wpi.first.units.Units.Radians;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

/**
 * Factory for an open-loop fallback command: apply a fixed percent output to
 * the arm motor until the arm angle reaches the requested target within a
 * tolerance.
 *
 * Prefer the closed-loop {@code arm.setAngle(...)} in normal operation. Use a
 * timeout when scheduling this command to avoid motor stall.
 */
/*public final class PowerUntilAngle {
  private PowerUntilAngle() {}

  public static Command create(frc.robot.subsystems.arm armSub, Angle target, double power, double toleranceRad) {
    // Runnable that applies the correct sign of open-loop output towards the
    // target. The command framework will call this periodically while active.
    Runnable apply = () -> {
      Angle current = armSub.getArmCurrentAngleSupplier().get();
  double error = target.in(Radians) - current.in(Radians);
      double sign = Math.signum(error);
      armSub.setOpenLoop(sign * Math.abs(power));
    };

    Runnable stop = () -> armSub.setOpenLoop(0.0);

    BooleanSupplier atTarget = () -> {
      Angle current = armSub.getArmCurrentAngleSupplier().get();
  return Math.abs(target.in(Radians) - current.in(Radians)) <= toleranceRad;
    };

    return Commands.runEnd(apply, stop, armSub).until(atTarget);
  }
}
*/