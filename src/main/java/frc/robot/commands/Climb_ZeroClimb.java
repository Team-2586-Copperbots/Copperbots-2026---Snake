package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.climb.Climb;

public class Climb_ZeroClimb extends Command {
    // Command to zero the climb subsystem's position at the start of auto or teleop
    private final Climb climb;

    public Climb_ZeroClimb(Climb climbSubsystem) {
        this.climb = climbSubsystem;
        addRequirements(climbSubsystem);
    }

    @Override
    public void initialize() {
        climb.setClimbSpeed(-0.5);
    }

    @Override
    public void execute() {
    }

    @Override
    public boolean isFinished() {
        return climb.getLimitSwitch();
    }

    @Override
    public void end(boolean interrupted) {
        climb.setClimbSpeed(0);
        if (!interrupted && climb.getLimitSwitch()) {
            climb.setPositionToZero();
        }
    }
}
