package frc.robot.commands;

import frc.robot.subsystems.IntakeSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj.Timer;

public class StartIntakeCommand extends Command {
    private final IntakeSubsystem intake;
    private final double durationSeconds;
    private final Timer timer = new Timer();

    public StartIntakeCommand(IntakeSubsystem intake, double durationSeconds) {
        this.intake = intake;
        this.durationSeconds = durationSeconds;
        addRequirements(intake);
    }

    @Override
    public void initialize() {
        intake.feedToShooter();
        timer.reset();
        timer.start();
        System.out.println("Intake running for " + durationSeconds + "s");
    }

    @Override
    public void execute() {}

    @Override
    public boolean isFinished() {
        return timer.hasElapsed(durationSeconds);
    }

    @Override
    public void end(boolean interrupted) {
        intake.feederOff();
        timer.stop();
        System.out.println("Intake stopped. Interrupted: " + interrupted);
    }
}
