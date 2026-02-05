package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.IntakeSubsystem.IntakePosition;

public class PIDIntake extends Command {
    private IntakeSubsystem Intake;
    private IntakePosition position;

    public PIDIntake(IntakeSubsystem Intake, IntakePosition position) {
        this.Intake = Intake;
        this.position = position;
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(Intake);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        Intake.setIntakePosition(position);
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {

    }

    @Override
    public boolean isFinished() {
        if (Math.abs(Intake.getMovementBarPosition() - position.getValue()) < 0.10) {
            return true;
        }
        return false;
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {

    }

}
