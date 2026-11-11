package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;

/**
 * Owns the ball-indexing path migrated out of the combined 2026 LaunchSubsystem:
 * the hopper flipper and the ground intake roller. The flywheels and climber now
 * live in {@link LaunchSubsystem}.
 */
public class IntakeSubsystem extends SubsystemBase {

    private final SparkMax m_flipperMotor =
            new SparkMax(IntakeConstants.FLIPPER_CAN_ID, MotorType.kBrushless);
    private final SparkMax m_intakeMotor =
            new SparkMax(IntakeConstants.MOTOR_CAN_ID, MotorType.kBrushless);

    private double lastTelemetryTime = Double.NEGATIVE_INFINITY;

    public IntakeSubsystem() {
    }

    // --- Low-level actuation ------------------------------------------------
    // Speeds come from IntakeConstants.FORWARD_OUTPUT / REVERSE_OUTPUT (public, so
    // they can be read or tuned from anywhere) instead of bare 1.0 / -1.0 literals.

    private void flipperForward() { m_flipperMotor.set(IntakeConstants.FORWARD_OUTPUT); }
    private void flipperBackward() { m_flipperMotor.set(IntakeConstants.REVERSE_OUTPUT); }

    private void intakeOn() { m_intakeMotor.set(IntakeConstants.FORWARD_OUTPUT); }
    private void intakeReverse() { m_intakeMotor.set(IntakeConstants.REVERSE_OUTPUT); }

    /** Runs the complete ball path toward the flywheels, without starting them. */
    public void feedToShooter() {
        flipperForward();
        intakeOn();
    }

    /** Reverses the complete ball path away from the flywheels and into the hopper. */
    public void returnToHopper() {
        flipperBackward();
        intakeReverse();
    }

    public void feederOff() {
        m_flipperMotor.stopMotor();
        m_intakeMotor.stopMotor();
    }

    // --- Command factories --------------------------------------------------

    /** Feeds the ball path toward the flywheels while scheduled, stopping on end. */
    public Command feedCommand() {
        return startEnd(this::feedToShooter, this::feederOff);
    }

    /** Reverses the ball path back into the hopper while scheduled, stopping on end. */
    public Command returnCommand() {
        return startEnd(this::returnToHopper, this::feederOff);
    }

    @Override
    public void periodic() {
        double now = Timer.getFPGATimestamp();
        if (now - lastTelemetryTime < 0.1) return;
        lastTelemetryTime = now;

        String key = IntakeConstants.DASHBOARD_PREFIX;
        SmartDashboard.putBoolean(key + "Feed Running",
                Math.abs(m_flipperMotor.get()) > 0.05 || Math.abs(m_intakeMotor.get()) > 0.05);
        SmartDashboard.putNumber(key + "Flipper Output", m_flipperMotor.get());
        SmartDashboard.putNumber(key + "Intake Output", m_intakeMotor.get());
    }
}
