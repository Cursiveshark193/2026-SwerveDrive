// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot; // root package for robot code

import frc.robot.Constants.OperatorConstants; // operator constants (joystick ports, deadbands)
import frc.robot.subsystems.ShooterSubsystem; // shooter subsystem
import frc.robot.subsystems.SwerveSubsystem; // swerve drive subsystem
import frc.robot.subsystems.arm;
import frc.robot.subsystems.conveyor;
import frc.robot.subsystems.intake;
import frc.robot.subsystems.FeederSubsystem; // example second mechanism subsystem for shooter feeder
import swervelib.SwerveInputStream; // helper to build swerve input streams
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d; // 2D rotation helper
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase; // robot base utility (simulation check)
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command; // WPILib command interface
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController; // Xbox controller wrapper for commands

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  /** The robot's swerve drive subsystem. */
  private final SwerveSubsystem drivebase = new SwerveSubsystem(); // instantiate the swerve subsystem

  /** The robot's shooter subsystem. */
  private final ShooterSubsystem m_Shooter = new ShooterSubsystem(); // instantiate the shooter subsystem
  private final conveyor m_conveyor = new conveyor(); // example second mechanism for conveyor (can also be in its own
                                                      // subsystem if desired)
  private final intake m_Intake = new intake(); // intake (disabled)
  private final arm m_arm = new arm();
  // private final FeederSubsystem m_ShooterFeeder = new FeederSubsystem(); //
  // example second mechanism for shooter feeder (can also be in its own subsystem
  // if desired)

  // private final Climber m_Climber = new Climber();
  private final FeederSubsystem m_ShooterFeeder = new FeederSubsystem(); // example second mechanism for shooter feeder
                                                                         // command
                                                                                                                       // that
                                                                                                                       // uses
                                                                                                                       // multiple
                                                                                                                       // subsystems
                                                                                                                       // (shooter,
                                                                                                                       // shooter
                                                                                                                       // feeder,
                                                                                                                       // and
                                                                                                                       // conveyor)
  private final SendableChooser<Command> autoChooser;
  // m_ShooterFeeder
  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandXboxController m_driverController = new CommandXboxController(
      OperatorConstants.kDriverControllerPort); // driver controller on configured port
  private final CommandXboxController m_operatorController = new CommandXboxController(
      OperatorConstants.kOperatorControllerPort); // operator controller on configured port

  /**
   * RobotContainer constructor. Creates subsystems, configures button bindings,
   * and sets default commands for subsystems.
   */
  public RobotContainer() {
    // Configure the trigger bindings
    DriverStation.silenceJoystickConnectionWarning(true);
    configureBindings(); // set up button->command mappings
    autoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Auto Chooser", autoChooser);
    // Set the default auto (do nothing)
    autoChooser.setDefaultOption("Do Nothing", Commands.none());

    // Add a simple auto option to have the robot drive forward for 1 second then
    // stop
    autoChooser.addOption("Drive Forward", drivebase.getAutonomousCommand("New Auto"));

    NamedCommands.registerCommand("test", Commands.print("Hello World"));

    // Set the default command to hold shooter at rest (0 RPM)
    m_Shooter.setDefaultCommand(m_Shooter.Stop()); // default command to stop shooter
    m_conveyor.setDefaultCommand(m_conveyor.StopConveyor()); // default command to stop conveyor
    m_Intake.setDefaultCommand(m_Intake.IntakeOff()); // intake angle default (disabled)
    // Choose a default drive command depending on whether we're in sim
    m_arm.setDefaultCommand(m_arm.set(0));
    m_ShooterFeeder.setDefaultCommand(m_ShooterFeeder.StopFeeder());
        
  }

    SwerveInputStream driveAngularVelocity = SwerveInputStream.of(drivebase.getSwerveDrive(),
                                                                () -> m_driverController.getLeftY() * -1,
                                                                () -> m_driverController.getLeftX() * -1)
                                                            .withControllerRotationAxis(m_driverController::getRightX)
                                                            .deadband(OperatorConstants.DEADBAND)
                                                            .scaleTranslation(0.8)
                                                            .allianceRelativeControl(true);

  /**
   * Clone's the angular velocity input stream and converts it to a fieldRelative input stream.
   */
  SwerveInputStream driveDirectAngle = driveAngularVelocity.copy().withControllerHeadingAxis(m_driverController::getRightX,
                                                                                             m_driverController::getRightY)
                                                           .headingWhile(true);

  /**
   * Clone's the angular velocity input stream and converts it to a robotRelative input stream.
   */
  SwerveInputStream driveRobotOriented = driveAngularVelocity.copy().robotRelative(true)
                                                             .allianceRelativeControl(false);

  SwerveInputStream driveAngularVelocityKeyboard = SwerveInputStream.of(drivebase.getSwerveDrive(),
                                                                        () -> -m_driverController.getLeftY(),
                                                                        () -> -m_driverController.getLeftX())
                                                                    .withControllerRotationAxis(() -> m_driverController.getRawAxis(
                                                                        2))
                                                                    .deadband(OperatorConstants.DEADBAND)
                                                                    .scaleTranslation(0.8)
                                                                    .allianceRelativeControl(true);
  // Derive the heading axis with math!
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
                                                                                                               2))
                                                                               .headingWhile(true)
                                                                               .translationHeadingOffset(true)
                                                                               .translationHeadingOffset(Rotation2d.fromDegrees(
                                                                                   0));
  /**
   * Use this method to define your trigger->command mappings. Triggers can be
   * created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with
   * an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for
   * {@link
   * CommandXboxController
   * Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or
   * {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */

  /**
   * Configure the trigger->command mappings. This hooks controller buttons
   * up to commands that operate subsystems (for example, spinning the shooter
   * or running the intake).
   */
  private void configureBindings() { 
     Command driveFieldOrientedDirectAngle      = drivebase.driveFieldOriented(driveDirectAngle);
    Command driveFieldOrientedAnglularVelocity = drivebase.driveFieldOriented(driveAngularVelocity);
    Command driveRobotOrientedAngularVelocity  = drivebase.driveFieldOriented(driveRobotOriented);
    Command driveSetpointGen = drivebase.driveWithSetpointGeneratorFieldRelative(
        driveDirectAngle);
    Command driveFieldOrientedDirectAngleKeyboard      = drivebase.driveFieldOriented(driveDirectAngleKeyboard);
    Command driveFieldOrientedAnglularVelocityKeyboard = drivebase.driveFieldOriented(driveAngularVelocityKeyboard);
    Command driveSetpointGenKeyboard = drivebase.driveWithSetpointGeneratorFieldRelative(
        driveDirectAngleKeyboard);
    
    // map controller inputs to commands
    m_operatorController.a().whileTrue(m_Intake.IntakeOn().alongWith(m_conveyor.ReverseConveyor())); // hold A on
                                                                                                     // operator
                                                                                                     // controller to
                                                                                                     // run intake at
                                                                                                     // 3000 RPM
    m_operatorController.b().whileTrue(m_Intake.ReverseIntake()); // hold B on operator controller to run intake in
                                                                  // reverse at 3000 RPM
    m_operatorController.rightBumper().whileTrue(Commands.sequence(
        Commands.waitSeconds(0.125)
            .andThen(m_Shooter.setDutyCycle(0.75))) // run shooter at 4000 RPM for 0.25 seconds to get up to speed
        .alongWith(
            Commands.parallel(
                // keep shooter running at 4000 RPM,
                Commands.waitSeconds(4)
                    .andThen(m_ShooterFeeder.ReverseFeeder().alongWith(m_conveyor.ReverseConveyor())
                        .alongWith(m_Intake.IntakeOn()
                            .alongWith(m_arm.set(0.15)
                                .withTimeout(0.5)
                                .andThen(m_arm.set(-0.15)
                                    .repeatedly()
                                    .withTimeout(0.5))
                                .repeatedly()))))));
    m_driverController.leftBumper().whileTrue(Commands.runOnce(drivebase::lock, drivebase).repeatedly());
    m_driverController.b().whileTrue(drivebase.driveToPose(new Pose2d(new Translation2d(2.880, 4), Rotation2d.fromDegrees(0)))); // hold B on driver controller to reset
                                                                          // heading to 0 degrees
    m_driverController.start().whileTrue(drivebase.centerModulesCommand());
    m_operatorController.leftBumper().whileTrue(Commands.parallel(m_ShooterFeeder.RunFeeder().alongWith(m_conveyor.RunConveyor())));
    m_operatorController.povDown().whileTrue(m_arm.set(-0.1)); // hold X to move arm to 45 degrees
    m_operatorController.povUp().whileTrue(m_arm.set(0.1)); // hold Y to move arm back to 0 degrees
    if m_driverController.back().getAsBoolean(){
        drivebase.
    }
  }

  /**
   * Returns the autonomous command to run during the autonomous period.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return new SequentialCommandGroup(
        
        autoChooser.getSelected().alongWith(
        m_arm.set(-0.15).withTimeout(0.25)),
        (Commands.sequence(
        Commands.waitSeconds(0.125)
            .andThen(m_Shooter.setDutyCycle(0.6))) // run shooter at 4000 RPM for 0.25 seconds to get up to speed
        .alongWith(
            Commands.parallel(
                // keep shooter running at 4000 RPM,
                Commands.waitSeconds(3)
                    .andThen(m_ShooterFeeder.ReverseFeeder().alongWith(m_conveyor.ReverseConveyor())
                        .alongWith(m_Intake.IntakeOn()
                            .alongWith(m_arm.set(0.15)
                                .withTimeout(0.5)
                                .andThen(m_arm.set(-0.15)
                                    .repeatedly()
                                    .withTimeout(0.5))
                                .repeatedly()))))))); // run the shooter/feeder/conveyor for 3 seconds

    // run the selected auto from the chooser
    // then run the shooter, feeder, and conveyor for 3 seconds;
    // run shooter for 3 seconds

    // An example command will be run in autonomous
    // return the example auto command (replace with your own command)
  }
public void setMotorBrake(boolean brake)
  {
    drivebase.setMotorBrake(brake);
  }
}
