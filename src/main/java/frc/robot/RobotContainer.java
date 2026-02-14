// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot; // root package for robot code

import frc.robot.Constants.OperatorConstants; // operator constants (joystick ports, deadbands)
import frc.robot.commands.Autos; // autonomous command factories
import frc.robot.subsystems.ShooterSubsystem; // shooter subsystem
import frc.robot.subsystems.SwerveSubsystem; // swerve drive subsystem
import frc.robot.subsystems.arm;
import frc.robot.subsystems.conveyor;
import frc.robot.subsystems.intake;
import swervelib.SwerveInputStream; // helper to build swerve input streams

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.RPM; // RPM unit helper

import edu.wpi.first.math.geometry.Rotation2d; // 2D rotation helper
import edu.wpi.first.wpilibj.RobotBase; // robot base utility (simulation check)
import edu.wpi.first.wpilibj2.command.Command; // WPILib command interface
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController; // Xbox controller wrapper for commands
import edu.wpi.first.wpilibj2.command.button.Trigger; // Trigger wrapper for boolean suppliers


/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  /** The robot's swerve drive subsystem. */
  private final SwerveSubsystem drivebase = new SwerveSubsystem(); // instantiate the swerve subsystem

  /** The robot's shooter subsystem. */
  private final ShooterSubsystem m_Shooter = new ShooterSubsystem(); // instantiate the shooter subsystem
  private final conveyor m_conveyor = new conveyor(); // example second mechanism for conveyor (can also be in its own subsystem if desired)
  private final intake m_Intake = new intake(); // intake (disabled)
  private final arm m_arm = new arm();

  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandXboxController m_driverController =
    new CommandXboxController(OperatorConstants.kDriverControllerPort); // driver controller on configured port

  /**
   * RobotContainer constructor. Creates subsystems, configures button bindings,
   * and sets default commands for subsystems.
   */
  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings(); // set up button->command mappings
    
    // Set the default command to hold shooter at rest (0 RPM)
     m_Shooter.setDefaultCommand(m_Shooter.setVelocity(RPM.of(0)));
    m_Intake.setDefaultCommand(m_Intake.IntakeOff()); // intake angle default (disabled)
    // Choose a default drive command depending on whether we're in sim
    m_arm.setDefaultCommand(m_arm.OnStandby()); // default command to hold arm at 0 degrees
    drivebase.setDefaultCommand(!RobotBase.isSimulation() ? driveFieldOrientedAngularVelocity : driveFieldOrientedDirectAngleKeyboard);
  }
  SwerveInputStream driveAngularVelocity = SwerveInputStream.of(drivebase.getSwerveDrive(), // build input stream from controller axes
                                                                () -> m_driverController.getLeftY() * -1, // forward/back (invert axis)
                                                                () -> m_driverController.getLeftX() * -1) // strafe (invert axis)
                                                            .withControllerRotationAxis(m_driverController::getRightX) // rotation from right stick X
                                                            .deadband(OperatorConstants.DEADBAND) // apply deadband
                                                            .scaleTranslation(0.8) // scale translational speed
                                                            .allianceRelativeControl(true); // make controls alliance-relative

  SwerveInputStream driveDirectAngle = driveAngularVelocity.copy().withControllerHeadingAxis(m_driverController::getRightX, // derive heading vector from right stick
                                                                                             m_driverController::getRightY)
                                                           .headingWhile(true); // hold heading while condition true
  
  Command driveFieldOrientedDirectAngle = drivebase.driveFieldOriented(driveDirectAngle);  // command for field-oriented driving using direct-angle input
  
  Command driveFieldOrientedAngularVelocity = drivebase.driveFieldOriented(driveAngularVelocity);  // command for field-oriented driving using angular velocity input
  
  SwerveInputStream driveAngularVelocityKeyboard = SwerveInputStream.of(drivebase.getSwerveDrive(), // keyboard/alternate mapping variant
                                    () -> -m_driverController.getLeftY(), // forward/back
                                    () -> -m_driverController.getLeftX()) // strafe
                                  .withControllerRotationAxis(() -> m_driverController.getRawAxis(
                                    2)) // rotation axis from raw axis 2
                                  .deadband(OperatorConstants.DEADBAND) // deadband
                                  .scaleTranslation(0.8) // translation scaling
                                  .allianceRelativeControl(true); // alliance relative
  // Derive the heading axis with math (alternate mapping for keyboard/controller)
  SwerveInputStream driveDirectAngleKeyboard     = driveAngularVelocityKeyboard.copy()
                                         .withControllerHeadingAxis(() ->
                                                        Math.sin(
                                                          m_driverController.getRawAxis(
                                                            2) *
                                                          Math.PI) *
                                                        (Math.PI *
                                                         2),
                                                      () ->
                                                        Math.cos(
                                                          m_driverController.getRawAxis(
                                                            2) *
                                                          Math.PI) *
                                                        (Math.PI *
                                                         2)) // heading vector math from raw axis
                                         .headingWhile(true) // hold heading while true
                                         .translationHeadingOffset(true) // enable translation heading offset
                                         .translationHeadingOffset(Rotation2d.fromDegrees(
                                           0)); // offset in degrees

Command driveFieldOrientedDirectAngleKeyboard = drivebase.driveFieldOriented(driveDirectAngleKeyboard); // drive command for keyboard/direct-angle

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  /**
   * Configure the trigger->command mappings. This hooks controller buttons
   * up to commands that operate subsystems (for example, spinning the shooter
   * or running the intake).
   */
  private void configureBindings() { // map controller inputs to commands
    
 m_driverController.a().onTrue(m_Intake.IntakeOn()); // example: run intake when A button is pressed
 m_driverController.start().and(m_driverController.a().onTrue(Commands.parallel(m_Intake.IntakeOff(),m_conveyor.StopConveyor()))); // stop intake when A button is released
 m_driverController.b().onTrue(m_Intake.ReverseIntake()); // example: reverse intake when B button is pressed
 m_driverController.b().onFalse(m_Intake.IntakeOff()); // stop intake when B button is released

 m_driverController.x().onTrue(m_Shooter.setVelocity(RPM.of(3000))); // example: set shooter to 3000 RPM when X button is pressed
 m_driverController.y().onTrue(m_Shooter.setVelocity(RPM.of(0))); // example: stop shooter when Y button is pressed

 m_driverController.leftBumper().onTrue(m_arm.Set_To_90_Degrees()); // example: set arm to 90 degrees when left bumper is pressed
 m_driverController.rightBumper().onTrue(m_arm.StowArm()); // example: stow arm at starting position when right bumper is pressed
 m_driverController.povUp().onTrue(m_arm.Agitate()); // example: agitate arm by moving to 10 degrees and back when D-pad up is pressed
 m_driverController.povDown().onTrue(m_arm.OnStandby()); // example: hold arm at 0 degrees when D-pad down is pressed
    // Map driver controller buttons to shooter commands
   
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  /**
   * Returns the autonomous command to run during the autonomous period.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return Autos.exampleAuto(drivebase);
  }
}
