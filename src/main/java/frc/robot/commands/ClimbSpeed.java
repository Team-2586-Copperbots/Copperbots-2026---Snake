package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.climb.Climb;

public class ClimbSpeed extends Command {
    private Climb climb;
    private Double Speed;

    public ClimbSpeed(Climb climbSubsystem, double speed) {
        this.climb = climbSubsystem;
        this.Speed = speed;

        addRequirements(climbSubsystem);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        climb.setClimbSpeed(Speed);
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
        climb.setClimbSpeed(0);
    }

}
