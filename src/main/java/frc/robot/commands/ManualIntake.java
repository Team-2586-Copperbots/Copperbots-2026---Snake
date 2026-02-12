package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem;

public class ManualIntake extends Command {
    private IntakeSubsystem intake;
    private Double speed;

    // positive is out
    public ManualIntake(IntakeSubsystem IntakeSubsystem, double speed) {
        this.intake = IntakeSubsystem;
        this.speed = speed;
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(IntakeSubsystem);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        intake.setMovementBarSpeed(speed);
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {

    }

    @Override
    public boolean isFinished() {
        return false;
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        intake.setMovementBarSpeed(0);
    }

}
