package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;

/**
 * Owns the ball-indexing path migrated out of the combined 2026 LaunchSubsystem:
 * the hopper flipper and the ground intake roller. The flywheels and climber now
 * live in {@link LaunchSubsystem}.
 */
public class IntakeSubsystem extends SubsystemBase {

    private final SparkMax flipperMotor = new SparkMax(5, MotorType.kBrushless);
    private final SparkMax intakeMotor = new SparkMax(IntakeConstants.MOTOR_CAN_ID, MotorType.kBrushless);

    private double lastTelemetryTime = Double.NEGATIVE_INFINITY;

    public IntakeSubsystem() {
    }

    public void flipperForward() { flipperMotor.set(-1.0); }
    public void flipperBackward() { flipperMotor.set(1.0); }
    public void flipperOff() { flipperMotor.stopMotor(); }

    public void intakeOn() { intakeMotor.set(1.0); }
    public void intakeReverse() { intakeMotor.set(-1.0); }
    public void intakeOff() { intakeMotor.stopMotor(); }

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
        flipperOff();
        intakeOff();
    }

    @Override
    public void periodic() {
        double now = Timer.getFPGATimestamp();
        if (now - lastTelemetryTime < 0.1) return;
        lastTelemetryTime = now;

        String key = IntakeConstants.DASHBOARD_PREFIX;
        SmartDashboard.putBoolean(key + "Feed Running",
                Math.abs(flipperMotor.get()) > 0.05 || Math.abs(intakeMotor.get()) > 0.05);
        SmartDashboard.putNumber(key + "Flipper Output", flipperMotor.get());
        SmartDashboard.putNumber(key + "Intake Output", intakeMotor.get());
    }
}
