package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.climb.Climb;
import frc.robot.subsystems.climb.Climb.ClimbPosition;

public class Climb_Move extends Command {
    // Class to move the climb subsystem up and down
    // Supports either an open-loop speed (-1.0 to 1.0) or a closed-loop ClimbPosition target
    private final Climb climb;
    private double speed = -2.0;
    private ClimbPosition position = null;

    public Climb_Move(Climb climbSubsystem, double speed) {
        this.climb = climbSubsystem;
        this.speed = speed;
        addRequirements(climbSubsystem);
    }

    public Climb_Move(Climb climbSubsystem, ClimbPosition position) {
        this.climb = climbSubsystem;
        this.position = position;
        addRequirements(climbSubsystem);
    }

    @Override
    public void initialize() {
        if (position != null) {
            climb.setClimbTargetPosition(position);
        } else if (speed != -2.0) {
            climb.setClimbSpeed(speed);
        }
    }

    @Override
    public void execute() {
        if (position != null) {
            climb.setClimbTargetPosition(position);
        }
    }

    @Override
    public boolean isFinished() {
        if (position != null) {
            return Math.abs(climb.getPosition() - position.value) < 0.1;
        }
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        if (position != null) {
            // Closed loop position target: maintain position holding if target reached, stop if interrupted
            if (!interrupted) {
                climb.setClimbTargetPosition(position);
            } else {
                climb.setClimbSpeed(0);
            }
        } else {
            climb.setClimbSpeed(0);
        }
    }
}
