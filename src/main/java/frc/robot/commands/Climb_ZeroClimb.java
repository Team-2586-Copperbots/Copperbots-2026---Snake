package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.climb.Climb;
import frc.robot.subsystems.climb.Climb.ClimbPosition;
import frc.robot.subsystems.turret.Turret;;

public class Climb_ZeroClimb extends Command {
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
