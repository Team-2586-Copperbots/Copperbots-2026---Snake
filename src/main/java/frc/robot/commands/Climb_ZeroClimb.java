package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.climb.Climb;;

public class Climb_ZeroClimb extends Command {
    // this is a command to zero the climb subsystem's position at the start of the match or at robot start up
    private Climb Climb;

    public Climb_ZeroClimb(Climb ClimbSubsystem) {
        this.Climb = ClimbSubsystem;
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(ClimbSubsystem);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        Climb.setClimbSpeed(-0.5);
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {

    }

    @Override
    public boolean isFinished() {
        if (Climb.getLimitSwitch()) {
            return true;
        }
        return false;
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        Climb.setClimbSpeed(0);
        Climb.setPositionToZero();
        // Climb.setClimbTargetPosition(ClimbPosition.DOWN);
    }

}
