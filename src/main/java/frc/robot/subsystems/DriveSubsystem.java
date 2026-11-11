package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.io.File;
import java.util.function.DoubleSupplier;
import swervelib.parser.SwerveParser;
import yams.mechanisms.config.SwerveDriveConfig;
import yams.mechanisms.swerve.SwerveDrive;
import yams.mechanisms.swerve.utility.SwerveInputStream;
import yams.motorcontrollers.SmartMotorControllerConfig.TelemetryVerbosity;

/**
 * The swerve drivetrain. The real work (module kinematics, odometry, motor control) is done by the
 * YAMS {@link SwerveDrive} object; this subsystem builds it from the deployed config files and
 * exposes the few operations the rest of the robot needs: driving, zeroing the gyro, and reading or
 * resetting the robot's position on the field.
 */
public class DriveSubsystem extends SubsystemBase {

  // The swerve drive library object. Not final because it is assigned inside the try block below.
  private SwerveDrive drive;
  // A field picture for the dashboard that shows where odometry thinks the robot is.
  private final Field2d field = new Field2d();

  /**
   * Builds the swerve drive from the JSON config files in the deploy folder.
   *
   * @throws RuntimeException if the config files are missing or invalid, so the robot fails loudly
   *     at startup instead of driving with a broken configuration
   */
  public DriveSubsystem() {
    // Settings that are not in the JSON files:
    var cfg = new SwerveDriveConfig()
        // Where odometry starts: x = 3 m, y = 3 m, facing 0 degrees.
        .withStartingPose(new Pose2d(3, 3, Rotation2d.kZero))
        // Links the drive to this subsystem, so commands made by the drive require this subsystem.
        .withSubsystem(this)
        // Publishes detailed drive data to NetworkTables under "SwerveDrive".
        .withTelemetry("SwerveDrive", TelemetryVerbosity.HIGH);
    try {
      // Reads the module/gyro/motor JSON files from src/main/deploy/swerve/base on the robot.
      SwerveParser.parse(new File(Filesystem.getDeployDirectory(), "swerve/base"));
      // Builds the actual drive object from the parsed files plus the settings above.
      drive = SwerveParser.createSwerveDrive(cfg);
    } catch (Exception e) {
      // Stop the robot program with a clear message if the swerve config can't be loaded.
      throw new RuntimeException("YAGSL configuration failed: " + e.getMessage(), e);
    }
    // Puts the field picture on the dashboard under the name "Field".
    SmartDashboard.putData("Field", field);
  }

  /**
   * Turns three joystick inputs into a stream of drive speeds. "Angular velocity" means the
   * rotation input sets how fast the robot spins, not which direction it faces.
   *
   * @param x forward/backward input, from -1 to 1 (positive is forward)
   * @param y left/right input, from -1 to 1 (positive is left)
   * @param rot rotation input, from -1 to 1 (positive is counterclockwise)
   * @return a stream that RobotContainer can add a deadband and field-relative control to
   */
  public SwerveInputStream getAngularVelocityStream(DoubleSupplier x, DoubleSupplier y,
                                                    DoubleSupplier rot) {
    return new SwerveInputStream(drive, x, y, rot);
  }

  /**
   * Makes a command that drives the robot using the given input stream, reading it every loop.
   * RobotContainer uses this as the default command, so it runs whenever nothing else is driving.
   *
   * @param stream the joystick input stream from {@link #getAngularVelocityStream}
   * @return the drive command
   */
  public Command drive(SwerveInputStream stream) {
    // SwerveInputStream is a Supplier<ChassisSpeeds> and handles the field-to-robot
    // conversion internally when alliance-relative control is enabled.
    return drive.drive(stream);
  }

  /**
   * Makes a command that resets the gyro, so the direction the robot faces right now becomes
   * "forward" for field-relative driving. Use it when the robot's idea of forward has drifted.
   *
   * @return a command that runs once and finishes
   */
  public Command zeroGyro() {
    return runOnce(() -> drive.zeroGyro());
  }

  /**
   * Returns where odometry thinks the robot is.
   *
   * @return the robot's position (meters) and heading on the field
   */
  public Pose2d getPose() {
    return drive.getPose();
  }

  /**
   * Tells odometry the robot is at a known position, for example at the start of an auto.
   *
   * @param pose the robot's actual position (meters) and heading
   */
  public void resetOdometry(Pose2d pose) {
    drive.resetOdometry(pose);
  }

  /**
   * Returns how fast the robot is moving, measured relative to the robot itself (not the field).
   *
   * @return forward, sideways, and rotational speed
   */
  public ChassisSpeeds getChassisSpeeds() {
    return drive.getRobotRelativeSpeed();
  }

  /**
   * Meant to drive at fixed robot-relative speeds (for example, for a path-following library).
   *
   * <p>NOTE: {@code drive.drive(...)} returns a Command (see {@link #drive}), and this method
   * throws that command away without scheduling it, so calling this method does not move the
   * robot.
   *
   * @param speeds the forward, sideways, and rotational speeds to drive at
   */
  public void driveRobotRelative(ChassisSpeeds speeds) {
    drive.drive(() -> speeds);
  }

  /** Runs every robot loop (every 20 ms). Publishes the robot's position to the dashboard. */
  @Override
  public void periodic() {
    // Sends the drive library's own telemetry.
    drive.updateTelemetry();
    // Moves the robot icon on the dashboard's field picture.
    field.setRobotPose(getPose());
    // Shows the position as plain numbers too (x and y in meters, heading in degrees).
    SmartDashboard.putNumber("Robot X", getPose().getX());
    SmartDashboard.putNumber("Robot Y", getPose().getY());
    SmartDashboard.putNumber("Robot Heading", getPose().getRotation().getDegrees());
  }

  /** Runs every loop only in simulation. Steps the simulated drivetrain physics forward. */
  @Override
  public void simulationPeriodic() {
    drive.simIterate();
  }
}
