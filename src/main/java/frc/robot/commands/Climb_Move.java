package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.climb.Climb;
import frc.robot.subsystems.climb.Climb.ClimbPosition;

public class Climb_Move extends Command {
    // this is a class to move the climb subsystem up and down
    // two constructors are provided so you can move the climb using a ClimbPosition
    // varialble or a speed -1 - 1 for the motors to run at

    private Climb climb;
    private double Speed = -2;
    private ClimbPosition position = null;

    public Climb_Move(Climb climbSubsystem, double speed) {
        this.climb = climbSubsystem;
        this.Speed = speed;

        addRequirements(climbSubsystem);
    }

    public Climb_Move(Climb climbSubsystem, ClimbPosition position) {
        this.climb = climbSubsystem;
        this.position = position;

        addRequirements(climbSubsystem);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        if (Speed != -2) {
            climb.setClimbSpeed(Speed);
        } else if (position != null) {
            climb.setClimbTargetPosition(position);
        }
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {

    }

    @Override
    public boolean isFinished() {
        if (position != null && (Math.abs(climb.getPosition() - position.value) < 0.1)) {
            return true;
        }
        return false;
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        climb.setClimbSpeed(0);
    }

}
