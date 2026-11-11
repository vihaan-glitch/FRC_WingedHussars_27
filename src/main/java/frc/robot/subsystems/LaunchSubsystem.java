package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.REVLibError;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkFlexConfig;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;

/**
 * Owns the flywheel launcher and climber migrated out of the combined 2026
 * LaunchSubsystem. The hopper flipper and ground intake roller now live in
 * {@link IntakeSubsystem}.
 *
 * <p>The two flywheel motors run velocity PID on the motor controllers themselves: the robot code
 * sends a target RPM, and each Spark Flex adjusts its own output to hold that speed. The PID gains
 * can be tuned live from the dashboard (see {@link #periodic()}).
 */
public class LaunchSubsystem extends SubsystemBase {

    // ----- Hardware -----
    // Two flywheel motors on Spark Flex controllers (CAN IDs 18 and 36).
    private final SparkFlex shooterMotor = new SparkFlex(18, MotorType.kBrushless);
    private final SparkFlex shooterMotor2 = new SparkFlex(36, MotorType.kBrushless);
    // Climber motor on a Spark MAX (CAN ID 6).
    private final SparkMax climberMotor = new SparkMax(6, MotorType.kBrushless);

    // Each motor's built-in encoder. getVelocity() returns RPM.
    private final RelativeEncoder shooterEncoder;
    private final RelativeEncoder shooterEncoder2;
    // Each motor controller's onboard PID controller, used to send it a target speed.
    private final SparkClosedLoopController closedLoopController;
    private final SparkClosedLoopController closedLoopController2;

    // ----- State -----
    // The flywheel speed we are currently asking for (RPM). 0 when stopped or unjamming.
    private double currentTargetRPM;
    // The PID and feedforward gains currently loaded on the motor controllers.
    // They start at the defaults in Constants and change when someone edits them on the dashboard.
    private double appliedKP = ShooterConstants.DEFAULT_KP;
    private double appliedKI = ShooterConstants.DEFAULT_KI;
    private double appliedKD = ShooterConstants.DEFAULT_KD;
    private double appliedKV = ShooterConstants.DEFAULT_KV;
    // True if the last configuration sent to both motors succeeded.
    private boolean shooterConfigurationOk;
    // True while in unjam mode. Dashboard PID changes are ignored while this is true.
    private boolean unjamConfigured;
    // Previous average speed and time, used to estimate flywheel acceleration in periodic().
    private double previousAverageRPM;
    private double previousSampleTime = Double.NaN; // NaN means "no sample yet"
    private double averageAccelerationRPMPerSecond;
    // When the dashboard was last updated (seconds), and how many updates have been sent.
    private double lastTelemetryTime = Double.NEGATIVE_INFINITY;
    private long telemetryUpdateCount;

    /**
     * Sets up the flywheels. Gets each motor's encoder and PID controller, then sends the full
     * configuration. At startup the motors are reset to factory defaults first and the settings
     * are saved to their flash memory, so they survive a power cycle.
     */
    public LaunchSubsystem() {
        shooterEncoder = shooterMotor.getEncoder();
        shooterEncoder2 = shooterMotor2.getEncoder();
        closedLoopController = shooterMotor.getClosedLoopController();
        closedLoopController2 = shooterMotor2.getClosedLoopController();
        configureShooterPID(ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    // ----- Flywheel control -----

    /**
     * Sets both flywheels to an absolute RPM using onboard velocity PID.
     *
     * @param requestedRPM the target speed. Invalid values (NaN, infinity) become 0, and the result
     *     is limited to between 0 and {@link ShooterConstants#MAX_RPM}.
     */
    public void setShooterRPM(double requestedRPM) {
        if (!Double.isFinite(requestedRPM)) requestedRPM = 0.0;
        currentTargetRPM = MathUtil.clamp(requestedRPM, 0.0, ShooterConstants.MAX_RPM);
        // Send the target to both motor controllers; each runs its own PID loop to reach it.
        closedLoopController.setSetpoint(currentTargetRPM, SparkBase.ControlType.kVelocity);
        closedLoopController2.setSetpoint(currentTargetRPM, SparkBase.ControlType.kVelocity);
        SmartDashboard.putNumber(ShooterConstants.DASHBOARD_PREFIX + "Active Target RPM", currentTargetRPM);
    }

    /**
     * Sets the flywheels to a fraction of max RPM.
     *
     * @param percent fraction of max RPM, from 0 to 1 (0.65 means 65%)
     */
    private void setShooterPercent(double percent) {
        setShooterRPM(percent * ShooterConstants.MAX_RPM);
    }

    /** Cuts power to both flywheels. They spin down on their own rather than braking. */
    public void shooterOff() {
        shooterMotor.stopMotor();
        shooterMotor2.stopMotor();
        currentTargetRPM = 0.0;
    }

    /**
     * Retained for autonomous code, where the input is a fraction of maximum RPM.
     *
     * @param percent fraction of max RPM, from 0 to 1
     */
    public void setShooterSpeed(double percent) {
        setShooterPercent(percent);
    }

    /**
     * Checks whether both flywheels are within the default tolerance of a fraction of max RPM.
     *
     * @param percent fraction of max RPM, from 0 to 1
     * @return true if both flywheels are close enough to that speed
     */
    public boolean isAtTarget(double percent) {
        return isAtTargetRPM(percent * ShooterConstants.MAX_RPM,
                ShooterConstants.DEFAULT_TOLERANCE_RPM);
    }

    /**
     * Checks whether both flywheels are within a tolerance of a target RPM.
     *
     * @param targetRPM the speed to compare against. A target of 0 or less always returns false,
     *     so a stopped shooter never reports "at target".
     * @param toleranceRPM how far off each flywheel is allowed to be (RPM)
     * @return true only if both flywheels are within the tolerance
     */
    public boolean isAtTargetRPM(double targetRPM, double toleranceRPM) {
        return targetRPM > 0.0 && Double.isFinite(targetRPM) && Double.isFinite(toleranceRPM)
                && Math.abs(targetRPM - shooterEncoder.getVelocity()) <= Math.abs(toleranceRPM)
                && Math.abs(targetRPM - shooterEncoder2.getVelocity()) <= Math.abs(toleranceRPM);
    }

    /**
     * Returns flywheel motor 1's speed. Not called anywhere yet.
     *
     * @return motor 1's speed in RPM
     */
    public double getShooterRPM() {
        return shooterEncoder.getVelocity();
    }

    // ----- PID tuning and configuration -----

    /**
     * Applies live dashboard gains only when they change. These updates are not
     * persisted, so a classroom typo is not burned into controller flash.
     *
     * <p>Gains outside the allowed ranges are rejected and the old gains are kept. The result is
     * shown on the dashboard as "PID Status".
     *
     * @param kP proportional gain (0 to 0.01)
     * @param kI integral gain (0 to 0.001)
     * @param kD derivative gain (0 to 0.1)
     * @param kV velocity feedforward in volts per RPM (0 to 0.01)
     */
    public void applyDashboardPID(double kP, double kI, double kD, double kV) {
        // Reject the whole set if any gain is invalid or out of range.
        if (!validGain(kP, 0.01) || !validGain(kI, 0.001)
                || !validGain(kD, 0.1) || !validGain(kV, 0.01)) {
            SmartDashboard.putString(ShooterConstants.DASHBOARD_PREFIX + "PID Status",
                    "Rejected: gains must be finite/nonnegative (P<=.01, I<=.001, D<=.1, kV<=.01)");
            return;
        }
        // Do nothing if the gains haven't changed. This runs every 0.1 s, and reconfiguring the
        // motors each time would flood the CAN bus.
        if (same(kP, appliedKP) && same(kI, appliedKI)
                && same(kD, appliedKD) && same(kV, appliedKV)) return;

        // Save the new gains, then send them to the motors (without saving to flash).
        appliedKP = kP;
        appliedKI = kI;
        appliedKD = kD;
        appliedKV = kV;
        configureShooterPID(ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
        SmartDashboard.putString(ShooterConstants.DASHBOARD_PREFIX + "PID Status",
                shooterConfigurationOk ? "Applied (temporary)" : "REV configuration error");
    }

    /**
     * Checks that a gain is a real number between 0 and a maximum.
     *
     * @param value the gain to check
     * @param maximum the largest allowed value
     * @return true if the gain is allowed
     */
    private static boolean validGain(double value, double maximum) {
        return Double.isFinite(value) && value >= 0.0 && value <= maximum;
    }

    /**
     * Compares two doubles with a tiny tolerance, since == on doubles can fail on rounding error.
     *
     * @param a first value
     * @param b second value
     * @return true if they are effectively equal
     */
    private static boolean same(double a, double b) {
        return Math.abs(a - b) < 1e-12;
    }

    /**
     * Sends the full flywheel configuration (current limit, inversion, PID, feedforward) to both
     * motors. Also leaves unjam mode.
     *
     * @param resetMode whether to reset the motors to factory defaults first
     * @param persistMode whether to save the settings to the motors' flash memory
     */
    private void configureShooterPID(ResetMode resetMode, PersistMode persistMode) {
        // Motor 1 is inverted and motor 2 is not, so the same positive target spins both
        // flywheels in the shooting direction.
        REVLibError first = shooterMotor.configure(shooterConfig(true), resetMode, persistMode);
        REVLibError second = shooterMotor2.configure(shooterConfig(false), resetMode, persistMode);
        shooterConfigurationOk = first == REVLibError.kOk && second == REVLibError.kOk;
        unjamConfigured = false;
    }

    /**
     * Builds the normal flywheel configuration using the currently applied gains.
     *
     * @param inverted whether this motor's direction is reversed
     * @return the configuration to send to one flywheel motor
     */
    private SparkFlexConfig shooterConfig(boolean inverted) {
        SparkFlexConfig config = new SparkFlexConfig();
        config.smartCurrentLimit(ShooterConstants.CURRENT_LIMIT_AMPS)
                // Scales output as if the battery were always at 12 V, so the flywheels behave
                // the same as the battery drains during a match.
                .voltageCompensation(12.0)
                .inverted(inverted);
        config.closedLoop
                // The PID loop measures speed with the motor's built-in encoder.
                .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                .pid(appliedKP, appliedKI, appliedKD)
                // The PID may use anywhere from full reverse to full forward output.
                .outputRange(-1.0, 1.0);
        // Feedforward: supplies the voltage needed to hold a given speed (kV volts per RPM),
        // so the PID only has to correct the remaining error.
        config.closedLoop.feedForward.kV(appliedKV);
        return config;
    }

    /**
     * Unjam mode: drives both flywheels backward at 6 V, ignoring the PID, to push a stuck ball
     * back out.
     *
     * <p>The first call also re-sends the current limit and inversion. Because it uses
     * kNoResetSafeParameters, every other setting (including the PID gains) stays as it was, so
     * this re-sends values the motors already have. It also sets {@code unjamConfigured}, which
     * pauses dashboard PID updates until {@link #restoreNormalMode()} runs.
     */
    public void unjamHighTorque() {
        if (!unjamConfigured) {
            SparkFlexConfig config = new SparkFlexConfig();
            config.smartCurrentLimit(ShooterConstants.CURRENT_LIMIT_AMPS).inverted(true);
            SparkFlexConfig config2 = new SparkFlexConfig();
            config2.smartCurrentLimit(ShooterConstants.CURRENT_LIMIT_AMPS).inverted(false);
            REVLibError first = shooterMotor.configure(config,
                    ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
            REVLibError second = shooterMotor2.configure(config2,
                    ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
            shooterConfigurationOk = first == REVLibError.kOk && second == REVLibError.kOk;
            unjamConfigured = true;
        }
        // Open-loop voltage: no speed target, just a fixed reverse push.
        shooterMotor.setVoltage(-6.0);
        shooterMotor2.setVoltage(-6.0);
        currentTargetRPM = 0.0;
    }

    /** Leaves unjam mode: re-sends the normal PID configuration and stops the flywheels. */
    public void restoreNormalMode() {
        configureShooterPID(ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
        shooterOff();
    }

    /**
     * Rumbles a controller while the flywheels are at speed, so the operator knows when to shoot.
     * Not called anywhere yet.
     *
     * @param controller the controller to rumble
     * @param percent the target speed as a fraction of max RPM
     */
    public void rumbleController(XboxController controller, double percent) {
        double rumble = isAtTarget(percent) ? 0.5 : 0.0;
        controller.setRumble(RumbleType.kLeftRumble, rumble);
        controller.setRumble(RumbleType.kRightRumble, rumble);
    }

    // ----- Climber -----
    // Full power up, full power down, or off. There are no limit switches or soft limits in code,
    // so the climber keeps moving until the button is released.

    /** Runs the climber up at full power. */
    public void climbUp() { climberMotor.set(1.0); }
    /** Runs the climber down at full power. */
    public void climbDown() { climberMotor.set(-1.0); }
    /** Stops the climber. */
    public void climbOff() { climberMotor.stopMotor(); }

    /** Stops the flywheels and the climber. Not called anywhere yet. */
    public void stopAll() {
        shooterOff();
        climbOff();
    }

    /**
     * Runs every robot loop (every 20 ms), but only does work every 0.1 s. Estimates flywheel
     * acceleration, picks up PID gains and tolerance typed into the dashboard, and publishes
     * flywheel telemetry.
     */
    @Override
    public void periodic() {
        // Skip until 0.1 s has passed since the last update.
        double now = Timer.getFPGATimestamp();
        if (now - lastTelemetryTime < 0.1) return;

        // Read both flywheel speeds and their average.
        double rpm1 = shooterEncoder.getVelocity();
        double rpm2 = shooterEncoder2.getVelocity();
        double averageRPM = (rpm1 + rpm2) / 2.0;
        // Acceleration = change in speed / change in time. Skipped on the first sample and after
        // gaps longer than 0.25 s, where the estimate would be meaningless.
        double dt = now - previousSampleTime;
        if (Double.isFinite(previousSampleTime) && dt > 0.0 && dt < 0.25) {
            averageAccelerationRPMPerSecond = (averageRPM - previousAverageRPM) / dt;
        }
        previousAverageRPM = averageRPM;
        previousSampleTime = now;
        lastTelemetryTime = now;

        String key = ShooterConstants.DASHBOARD_PREFIX;
        // Read the PID gains from the dashboard and apply them if they changed (not during unjam).
        // If a key doesn't exist on the dashboard, the default from ShooterConstants is used.
        if (!unjamConfigured) {
            applyDashboardPID(
                    SmartDashboard.getNumber(key + "kP", ShooterConstants.DEFAULT_KP),
                    SmartDashboard.getNumber(key + "kI", ShooterConstants.DEFAULT_KI),
                    SmartDashboard.getNumber(key + "kD", ShooterConstants.DEFAULT_KD),
                    SmartDashboard.getNumber(key + "kV Volts per RPM", ShooterConstants.DEFAULT_KV));
        }

        // Read the "at target" tolerance from the dashboard (same default rule as above).
        double tolerance = SmartDashboard.getNumber(key + "Tolerance RPM",
                ShooterConstants.DEFAULT_TOLERANCE_RPM);
        boolean atTarget = isAtTargetRPM(currentTargetRPM, tolerance);

        // --- Telemetry ---
        // Heartbeat: shows the code is running and how many updates have been sent.
        SmartDashboard.putBoolean(key + "Tuner Online", true);
        SmartDashboard.putNumber(key + "Telemetry Update Count", ++telemetryUpdateCount);
        // Speeds, and how far each flywheel is from the target.
        SmartDashboard.putNumber(key + "Motor 1 RPM", rpm1);
        SmartDashboard.putNumber(key + "Motor 2 RPM", rpm2);
        SmartDashboard.putNumber(key + "Average RPM", averageRPM);
        SmartDashboard.putNumber(key + "Motor RPM Difference", rpm1 - rpm2);
        SmartDashboard.putNumber(key + "Active Target RPM", currentTargetRPM);
        SmartDashboard.putNumber(key + "Average Error RPM", currentTargetRPM - averageRPM);
        SmartDashboard.putNumber(key + "Motor 1 Error RPM", currentTargetRPM - rpm1);
        SmartDashboard.putNumber(key + "Motor 2 Error RPM", currentTargetRPM - rpm2);
        SmartDashboard.putNumber(key + "Acceleration RPM per sec", averageAccelerationRPMPerSecond);
        SmartDashboard.putBoolean(key + "At Target", atTarget);
        // Configuration status and the targets the motor controllers actually received.
        SmartDashboard.putBoolean(key + "PID Config OK", shooterConfigurationOk);
        SmartDashboard.putNumber(key + "Controller 1 Setpoint RPM", closedLoopController.getSetpoint());
        SmartDashboard.putNumber(key + "Controller 2 Setpoint RPM", closedLoopController2.getSetpoint());
        // The gains currently loaded on the motors.
        SmartDashboard.putNumber(key + "Applied kP", appliedKP);
        SmartDashboard.putNumber(key + "Applied kI", appliedKI);
        SmartDashboard.putNumber(key + "Applied kD", appliedKD);
        SmartDashboard.putNumber(key + "Applied kV Volts per RPM", appliedKV);
        // Electrical data: duty cycle (-1 to 1), output volts (duty cycle x battery voltage),
        // battery voltage at the controller, current, and temperature.
        SmartDashboard.putNumber(key + "Motor 1 Duty Cycle", shooterMotor.getAppliedOutput());
        SmartDashboard.putNumber(key + "Motor 2 Duty Cycle", shooterMotor2.getAppliedOutput());
        SmartDashboard.putNumber(key + "Motor 1 Volts",
                shooterMotor.getAppliedOutput() * shooterMotor.getBusVoltage());
        SmartDashboard.putNumber(key + "Motor 2 Volts",
                shooterMotor2.getAppliedOutput() * shooterMotor2.getBusVoltage());
        SmartDashboard.putNumber(key + "Motor 1 Bus Voltage", shooterMotor.getBusVoltage());
        SmartDashboard.putNumber(key + "Motor 2 Bus Voltage", shooterMotor2.getBusVoltage());
        SmartDashboard.putNumber(key + "Motor 1 Current Amps", shooterMotor.getOutputCurrent());
        SmartDashboard.putNumber(key + "Motor 2 Current Amps", shooterMotor2.getOutputCurrent());
        SmartDashboard.putNumber(key + "Motor 1 Temperature C", shooterMotor.getMotorTemperature());
        SmartDashboard.putNumber(key + "Motor 2 Temperature C", shooterMotor2.getMotorTemperature());
        // True if a motor controller reports a fault or warning (check REV Hardware Client for details).
        SmartDashboard.putBoolean(key + "Motor 1 Fault", shooterMotor.hasActiveFault());
        SmartDashboard.putBoolean(key + "Motor 2 Fault", shooterMotor2.hasActiveFault());
        SmartDashboard.putBoolean(key + "Motor 1 Warning", shooterMotor.hasActiveWarning());
        SmartDashboard.putBoolean(key + "Motor 2 Warning", shooterMotor2.hasActiveWarning());
        // The climber's current duty cycle (-1 to 1).
        SmartDashboard.putNumber("Climber Output", climberMotor.get());
    }
}
