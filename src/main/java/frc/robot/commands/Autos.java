// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LaunchSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

public final class Autos {
  /** Spins the flywheels up, feeds the indexer to fire, then stops everything. */
  public static Command spinUpAndFire(LaunchSubsystem launch, IntakeSubsystem intake) {
    return Commands.sequence(
        Commands.runOnce(() -> launch.setShooterSpeed(0.65), launch),
        Commands.waitSeconds(1.5),
        Commands.startEnd(intake::feedToShooter, intake::feederOff, intake).withTimeout(2.0),
        Commands.runOnce(launch::shooterOff, launch));
  }

  private Autos() {
    throw new UnsupportedOperationException("This is a utility class!");
  }
}
