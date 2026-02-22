// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot; // root package for robot code

import frc.robot.Constants.OperatorConstants; // operator constants (joystick ports, deadbands)
import frc.robot.commands.Autos; // autonomous command factories
import frc.robot.commands.RunShooterFeederConveyor;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.ShooterSubsystem; // shooter subsystem
import frc.robot.subsystems.SwerveSubsystem; // swerve drive subsystem
import frc.robot.subsystems.arm;
import frc.robot.subsystems.conveyor;
import frc.robot.subsystems.intake;
import yams.mechanisms.positional.Arm;
import yams.mechanisms.positional.Elevator;
//import frc.robot.subsystems.FeederSubsystem; // example second mechanism subsystem for shooter feeder
import swervelib.SwerveInputStream; // helper to build swerve input streams

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.RPM; // RPM unit helper
import static edu.wpi.first.units.Units.Radians;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Rotation2d; // 2D rotation helper
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.RobotBase; // robot base utility (simulation check)
import edu.wpi.first.wpilibj2.command.Command; // WPILib command interface
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController; // Xbox controller wrapper for commands


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
  //private final Climber m_Climber = new Climber();
//  private final FeederSubsystem m_ShooterFeeder = new FeederSubsystem(); // example second mechanism for shooter feeder (can also be in its own subsystem if desired)
  private final RunShooterFeederConveyor m_exampleCommand = new RunShooterFeederConveyor(m_Shooter, m_conveyor); // example command that uses multiple subsystems (shooter, shooter feeder, and conveyor)
 //m_ShooterFeeder,
  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandXboxController m_driverController =
    new CommandXboxController(OperatorConstants.kDriverControllerPort); // driver controller on configured port
  private final CommandXboxController m_operatorController =
    new CommandXboxController(OperatorConstants.kOperatorControllerPort); // operator controller on configured port

  /**
   * RobotContainer constructor. Creates subsystems, configures button bindings,
   * and sets default commands for subsystems.
   */
  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings(); // set up button->command mappings
    
    // Set the default command to hold shooter at rest (0 RPM)
     m_Shooter.setDefaultCommand(m_Shooter.Stop()); // default command to stop shooter
     m_conveyor.setDefaultCommand(m_conveyor.StopConveyor()); // default command to stop conveyor
    m_Intake.setDefaultCommand(m_Intake.IntakeOff()); // intake angle default (disabled)
    // Choose a default drive command depending on whether we're in sim
    //m_ShooterFeeder.setDefaultCommand(m_ShooterFeeder.Stop());
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
    m_operatorController.a().whileTrue(m_Intake.IntakeOn(RPM.of(1500))); // operator A: run intake at 3000 RPM while held
    m_operatorController.b().whileTrue(m_Intake.ReverseIntake()); // operator B: run intake in reverse at 30% while held
  // Driver X: move arm to 90° using YAMS closed-loop position command (Option A)
    m_operatorController.x().onTrue(m_arm.setAngle(Degrees.of(45)));
    m_operatorController.y().whileTrue(m_arm.set(0.1)); // operator Y: stow arm at starting position while held
  // Driver X: move arm to 90° using open-loop fallback command (Option B, not recommended unless you have a good reason to avoid closed-loop control)
  //  m_driverController.x().onTrue(m_arm.Set_To_90_Degrees()); 
    //m_operatorController.y().whileTrue(m_arm.Agitate()); // operator Y: stow arm at starting position while held
    m_operatorController.rightBumper().onTrue(new RunShooterFeederConveyor(m_Shooter, m_conveyor));     //, m_ShooterFeeder,
    m_operatorController.rightBumper().onFalse(m_Shooter.Stop()); // Y: stop shooter while held 
    m_operatorController.leftBumper().onTrue(m_conveyor.ReverseConveyor());
    //m_operatorController.povUp().whileTrue(m_Climber.set(0.1)); // left trigger: run climber at 50% while held
   // m_operatorController.povDown().onTrue(m_Climber.setHeightAndStop(Inches.of(12))); // right trigger: run climber in reverse at 50% while held
    // Map driver controller buttons to shooter commands
   
  }

  
  /**
   * Returns the autonomous command to run during the autonomous period.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return Autos.exampleAuto(m_Shooter, 
    //m_ShooterFeeder, 
    m_conveyor); // return the example auto command (replace with your own command)
  }

  public void teleoperatedInit() {
    m_arm.setAngleSetpoint(Degrees.of(0)); // reset arm to starting position at the beginning of teleop
    // Any initialization code for teleop can go here. For example, you could reset sensors or set initial subsystem states.
  

 
  }}
