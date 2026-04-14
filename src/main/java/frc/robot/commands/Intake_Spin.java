
package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.intake.Intake;

public class Intake_Spin extends Command {
    // command that only sets the speed of the intake roller, may or may not
    // interrupt the rest of the subsystem or don't thing it does
    private Intake intake;
    private double speed;
    private boolean stop;

    public Intake_Spin(Intake IntakeSubsystem, double speed, boolean stop) {
        this.intake = IntakeSubsystem;
        this.speed = speed;
        this.stop = stop;
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(IntakeSubsystem);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        intake.setRollerSpeed(speed);
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {

    }

    @Override
    public boolean isFinished() {
        if (stop) {
            return false;
        }
        return true;
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        if (stop) {
            intake.setRollerSpeed(0);
        }
    }

}
