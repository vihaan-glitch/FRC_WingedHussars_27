// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.Autos;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LaunchSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final LaunchSubsystem m_launch = new LaunchSubsystem();
  private final IntakeSubsystem m_intake = new IntakeSubsystem();

  // Operator runs the flywheels, indexer, and climber.
  // (The driver controller / drivetrain bindings are not wired up yet.)
  private final CommandXboxController m_operatorController =
      new CommandXboxController(OperatorConstants.kOperatorControllerPort);

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
   */
  private void configureBindings() {
    // Right trigger: spin the flywheels up to a shooting speed; coast down on release.
    m_operatorController.rightTrigger()
        .whileTrue(Commands.startEnd(() -> m_launch.setShooterSpeed(0.65), m_launch::shooterOff, m_launch));

    // Right bumper: run the indexer to feed balls into the flywheels.
    m_operatorController.rightBumper()
        .whileTrue(Commands.startEnd(m_intake::feedToShooter, m_intake::feederOff, m_intake));

    // Left bumper: reverse the indexer back into the hopper (clear a feed jam).
    m_operatorController.leftBumper()
        .whileTrue(Commands.startEnd(m_intake::returnToHopper, m_intake::feederOff, m_intake));

    // Y: high-torque flywheel unjam; restore velocity PID on release.
    m_operatorController.y()
        .whileTrue(Commands.startEnd(m_launch::unjamHighTorque, m_launch::restoreNormalMode, m_launch));

    // D-pad up/down: run the climber up or down.
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
    return Autos.spinUpAndFire(m_launch, m_intake);
  }
}
