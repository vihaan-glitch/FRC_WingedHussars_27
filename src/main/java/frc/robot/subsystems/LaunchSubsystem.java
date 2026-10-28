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
 */
public class LaunchSubsystem extends SubsystemBase {

    private final SparkFlex shooterMotor = new SparkFlex(18, MotorType.kBrushless);
    private final SparkFlex shooterMotor2 = new SparkFlex(36, MotorType.kBrushless);
    private final SparkMax climberMotor = new SparkMax(6, MotorType.kBrushless);

    private final RelativeEncoder shooterEncoder;
    private final RelativeEncoder shooterEncoder2;
    private final SparkClosedLoopController closedLoopController;
    private final SparkClosedLoopController closedLoopController2;

    private double currentTargetRPM;
    private double appliedKP = ShooterConstants.DEFAULT_KP;
    private double appliedKI = ShooterConstants.DEFAULT_KI;
    private double appliedKD = ShooterConstants.DEFAULT_KD;
    private double appliedKV = ShooterConstants.DEFAULT_KV;
    private boolean shooterConfigurationOk;
    private boolean unjamConfigured;
    private double previousAverageRPM;
    private double previousSampleTime = Double.NaN;
    private double averageAccelerationRPMPerSecond;
    private double lastTelemetryTime = Double.NEGATIVE_INFINITY;
    private long telemetryUpdateCount;

    public LaunchSubsystem() {
        shooterEncoder = shooterMotor.getEncoder();
        shooterEncoder2 = shooterMotor2.getEncoder();
        closedLoopController = shooterMotor.getClosedLoopController();
        closedLoopController2 = shooterMotor2.getClosedLoopController();
        configureShooterPID(ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    /** Sets both flywheels to an absolute RPM using onboard velocity PID. */
    public void setShooterRPM(double requestedRPM) {
        if (!Double.isFinite(requestedRPM)) requestedRPM = 0.0;
        currentTargetRPM = MathUtil.clamp(requestedRPM, 0.0, ShooterConstants.MAX_RPM);
        closedLoopController.setSetpoint(currentTargetRPM, SparkBase.ControlType.kVelocity);
        closedLoopController2.setSetpoint(currentTargetRPM, SparkBase.ControlType.kVelocity);
        SmartDashboard.putNumber(ShooterConstants.DASHBOARD_PREFIX + "Active Target RPM", currentTargetRPM);
    }

    private void setShooterPercent(double percent) {
        setShooterRPM(percent * ShooterConstants.MAX_RPM);
    }

    public void shooterOff() {
        shooterMotor.stopMotor();
        shooterMotor2.stopMotor();
        currentTargetRPM = 0.0;
    }

    /** Retained for autonomous code, where the input is a fraction of maximum RPM. */
    public void setShooterSpeed(double percent) {
        setShooterPercent(percent);
    }

    public boolean isAtTarget(double percent) {
        return isAtTargetRPM(percent * ShooterConstants.MAX_RPM,
                ShooterConstants.DEFAULT_TOLERANCE_RPM);
    }

    public boolean isAtTargetRPM(double targetRPM, double toleranceRPM) {
        return targetRPM > 0.0 && Double.isFinite(targetRPM) && Double.isFinite(toleranceRPM)
                && Math.abs(targetRPM - shooterEncoder.getVelocity()) <= Math.abs(toleranceRPM)
                && Math.abs(targetRPM - shooterEncoder2.getVelocity()) <= Math.abs(toleranceRPM);
    }

    public double getShooterRPM() {
        return shooterEncoder.getVelocity();
    }

    /**
     * Applies live dashboard gains only when they change. These updates are not
     * persisted, so a classroom typo is not burned into controller flash.
     */
    public void applyDashboardPID(double kP, double kI, double kD, double kV) {
        if (!validGain(kP, 0.01) || !validGain(kI, 0.001)
                || !validGain(kD, 0.1) || !validGain(kV, 0.01)) {
            SmartDashboard.putString(ShooterConstants.DASHBOARD_PREFIX + "PID Status",
                    "Rejected: gains must be finite/nonnegative (P<=.01, I<=.001, D<=.1, kV<=.01)");
            return;
        }
        if (same(kP, appliedKP) && same(kI, appliedKI)
                && same(kD, appliedKD) && same(kV, appliedKV)) return;

        appliedKP = kP;
        appliedKI = kI;
        appliedKD = kD;
        appliedKV = kV;
        configureShooterPID(ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
        SmartDashboard.putString(ShooterConstants.DASHBOARD_PREFIX + "PID Status",
                shooterConfigurationOk ? "Applied (temporary)" : "REV configuration error");
    }

    private static boolean validGain(double value, double maximum) {
        return Double.isFinite(value) && value >= 0.0 && value <= maximum;
    }

    private static boolean same(double a, double b) {
        return Math.abs(a - b) < 1e-12;
    }

    private void configureShooterPID(ResetMode resetMode, PersistMode persistMode) {
        REVLibError first = shooterMotor.configure(shooterConfig(true), resetMode, persistMode);
        REVLibError second = shooterMotor2.configure(shooterConfig(false), resetMode, persistMode);
        shooterConfigurationOk = first == REVLibError.kOk && second == REVLibError.kOk;
        unjamConfigured = false;
    }

    private SparkFlexConfig shooterConfig(boolean inverted) {
        SparkFlexConfig config = new SparkFlexConfig();
        config.smartCurrentLimit(ShooterConstants.CURRENT_LIMIT_AMPS)
                .voltageCompensation(12.0)
                .inverted(inverted);
        config.closedLoop
                .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                .pid(appliedKP, appliedKI, appliedKD)
                .outputRange(-1.0, 1.0);
        config.closedLoop.feedForward.kV(appliedKV);
        return config;
    }

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
        shooterMotor.setVoltage(-6.0);
        shooterMotor2.setVoltage(-6.0);
        currentTargetRPM = 0.0;
    }

    public void restoreNormalMode() {
        configureShooterPID(ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
        shooterOff();
    }

    public void rumbleController(XboxController controller, double percent) {
        double rumble = isAtTarget(percent) ? 0.5 : 0.0;
        controller.setRumble(RumbleType.kLeftRumble, rumble);
        controller.setRumble(RumbleType.kRightRumble, rumble);
    }

    public void climbUp() { climberMotor.set(1.0); }
    public void climbDown() { climberMotor.set(-1.0); }
    public void climbOff() { climberMotor.stopMotor(); }

    public void stopAll() {
        shooterOff();
        climbOff();
    }

    @Override
    public void periodic() {
        double now = Timer.getFPGATimestamp();
        if (now - lastTelemetryTime < 0.1) return;

        double rpm1 = shooterEncoder.getVelocity();
        double rpm2 = shooterEncoder2.getVelocity();
        double averageRPM = (rpm1 + rpm2) / 2.0;
        double dt = now - previousSampleTime;
        if (Double.isFinite(previousSampleTime) && dt > 0.0 && dt < 0.25) {
            averageAccelerationRPMPerSecond = (averageRPM - previousAverageRPM) / dt;
        }
        previousAverageRPM = averageRPM;
        previousSampleTime = now;
        lastTelemetryTime = now;

        String key = ShooterConstants.DASHBOARD_PREFIX;
        if (!unjamConfigured) {
            applyDashboardPID(
                    SmartDashboard.getNumber(key + "kP", ShooterConstants.DEFAULT_KP),
                    SmartDashboard.getNumber(key + "kI", ShooterConstants.DEFAULT_KI),
                    SmartDashboard.getNumber(key + "kD", ShooterConstants.DEFAULT_KD),
                    SmartDashboard.getNumber(key + "kV Volts per RPM", ShooterConstants.DEFAULT_KV));
        }

        double tolerance = SmartDashboard.getNumber(key + "Tolerance RPM",
                ShooterConstants.DEFAULT_TOLERANCE_RPM);
        boolean atTarget = isAtTargetRPM(currentTargetRPM, tolerance);

        SmartDashboard.putBoolean(key + "Tuner Online", true);
        SmartDashboard.putNumber(key + "Telemetry Update Count", ++telemetryUpdateCount);
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
        SmartDashboard.putBoolean(key + "PID Config OK", shooterConfigurationOk);
        SmartDashboard.putNumber(key + "Controller 1 Setpoint RPM", closedLoopController.getSetpoint());
        SmartDashboard.putNumber(key + "Controller 2 Setpoint RPM", closedLoopController2.getSetpoint());
        SmartDashboard.putNumber(key + "Applied kP", appliedKP);
        SmartDashboard.putNumber(key + "Applied kI", appliedKI);
        SmartDashboard.putNumber(key + "Applied kD", appliedKD);
        SmartDashboard.putNumber(key + "Applied kV Volts per RPM", appliedKV);
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
        SmartDashboard.putBoolean(key + "Motor 1 Fault", shooterMotor.hasActiveFault());
        SmartDashboard.putBoolean(key + "Motor 2 Fault", shooterMotor2.hasActiveFault());
        SmartDashboard.putBoolean(key + "Motor 1 Warning", shooterMotor.hasActiveWarning());
        SmartDashboard.putBoolean(key + "Motor 2 Warning", shooterMotor2.hasActiveWarning());
        SmartDashboard.putNumber("Climber Output", climberMotor.get());
    }
}
