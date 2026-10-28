// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
    public static final int kOperatorControllerPort = 1;
  }

  /** Safe starting values for the dashboard-driven shooter PID exercise (from 2026 robot). */
  public static class ShooterConstants {
    // Keep these keys flat so classic SmartDashboard displays them as widgets.
    public static final String DASHBOARD_PREFIX = "Shooter PID - ";

    public static final int CURRENT_LIMIT_AMPS = 95;
    public static final double MAX_RPM = 6784.0;
    public static final double DEFAULT_TOLERANCE_RPM = 150.0;

    public static final double DEFAULT_KP = 0.0003;
    public static final double DEFAULT_KI = 0.0;
    public static final double DEFAULT_KD = 0.001;
    // REVLib 2026 kV is in volts per RPM, so this reaches 12 V at free speed.
    public static final double DEFAULT_KV = 12.0 / MAX_RPM;
  }

  /** Intake roller constants. */
  public static class IntakeConstants {
    public static final int MOTOR_CAN_ID = 15;
    public static final String DASHBOARD_PREFIX = "Intake/";
  }
}
