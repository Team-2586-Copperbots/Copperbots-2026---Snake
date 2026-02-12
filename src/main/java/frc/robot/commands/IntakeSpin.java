
package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class IntakeSpin extends Command {
    private IntakeSubsystem intake;
    private Double speed;

    public IntakeSpin(IntakeSubsystem IntakeSubsystem, double speed) {
        this.intake = IntakeSubsystem;
        this.speed = speed;
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(IntakeSubsystem);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        intake.setSpinnerSpeed(speed);
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
        intake.setSpinnerSpeed(0);
    }

}
