// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.Autos;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LaunchSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import yams.mechanisms.swerve.utility.SwerveInputStream;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  // Each subsystem is created exactly once, here. Creating one also creates its motors.
  private final LaunchSubsystem m_launch = new LaunchSubsystem(); // flywheels + climber
  private final IntakeSubsystem m_intake = new IntakeSubsystem(); // flipper + intake roller
  private final DriveSubsystem m_drive = new DriveSubsystem();    // swerve drivetrain

  // Driver drives the swerve chassis; operator runs the flywheels, indexer, and climber.
  // The ports (0 and 1) are the controllers' slots in the Driver Station USB tab.
  private final CommandXboxController m_driverController =
      new CommandXboxController(OperatorConstants.kDriverControllerPort);
  private final CommandXboxController m_operatorController =
      new CommandXboxController(OperatorConstants.kOperatorControllerPort);

  // Field-relative drive from the left stick (translation) and right stick (rotation).
  // The stick values are negated because pushing a stick up or left reads as negative on an
  // Xbox controller, while the robot treats forward, left, and counterclockwise as positive.
  private final SwerveInputStream m_driveAngularVelocity =
      m_drive.getAngularVelocityStream(
                () -> -m_driverController.getLeftY(),   // forward/backward
                () -> -m_driverController.getLeftX(),   // left/right
                () -> -m_driverController.getRightX())  // rotation speed
            // Ignore stick inputs smaller than 5%, so a stick that doesn't center perfectly
            // doesn't make the robot creep.
            .withDeadband(0.05)
            // "Forward" means away from your own alliance wall, on both red and blue.
            .withAllianceRelativeControl();

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   *
   * <p>How the bindings below work: whileTrue(command) starts the command when the button is
   * pressed and cancels it when released; onTrue(command) starts it once per press.
   * Commands.startEnd(start, end, subsystem) runs start when it begins and end when it is canceled.
   * The last argument is the subsystem the command requires: only one command can use a subsystem
   * at a time, so starting a second one cancels the first.
   */
  private void configureBindings() {
    // Default drive command: field-relative teleop driving.
    // A default command runs whenever no other command is using the subsystem.
    m_drive.setDefaultCommand(m_drive.drive(m_driveAngularVelocity));

    // ----- Driver controller -----

    // Zero the gyro with Start + Back. Use this if the field-relative heading drifts.
    // Requiring both buttons makes it hard to press by accident.
    m_driverController.start().and(m_driverController.back()).onTrue(m_drive.zeroGyro());

    // ----- Operator controller -----

    // Right trigger: spin the flywheels up to a shooting speed; coast down on release.
    // 0.65 means 65% of the flywheels' max RPM.
    m_operatorController.rightTrigger()
        .whileTrue(Commands.startEnd(() -> m_launch.setShooterSpeed(0.65), m_launch::shooterOff, m_launch));

    // Right bumper: run the indexer to feed balls into the flywheels.
    m_operatorController.rightBumper().whileTrue(m_intake.feedCommand());

    // Left bumper: reverse the indexer back into the hopper (clear a feed jam).
    m_operatorController.leftBumper().whileTrue(m_intake.returnCommand());

    // Y: high-torque flywheel unjam; restore velocity PID on release.
    m_operatorController.y()
        .whileTrue(Commands.startEnd(m_launch::unjamHighTorque, m_launch::restoreNormalMode, m_launch));

    // D-pad up/down: run the climber up or down.
    // NOTE: the climber is part of m_launch, so these commands require the same subsystem as the
    // flywheel commands. Pressing the D-pad while holding the right trigger cancels the flywheels.
    m_operatorController.povUp()
        .whileTrue(Commands.startEnd(m_launch::climbUp, m_launch::climbOff, m_launch));
    m_operatorController.povDown()
        .whileTrue(Commands.startEnd(m_launch::climbDown, m_launch::climbOff, m_launch));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // Spins up the flywheels, feeds balls for 2 seconds, then stops (see Autos.java).
    return Autos.spinUpAndFire(m_launch, m_intake);
  }
}
