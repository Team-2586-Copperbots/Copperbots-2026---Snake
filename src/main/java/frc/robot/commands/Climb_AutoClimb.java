package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.lib.BLine.Path;
import frc.robot.subsystems.climb.Climb;
import frc.robot.subsystems.climb.Climb.ClimbPosition;
import frc.robot.subsystems.drive.Drive;
import frc.robot.util.driveUtils.ClimbUtils;

public class Climb_AutoClimb extends Command {
    private Climb climb;
    private Drive drive;
    private Command sequence;

    public Climb_AutoClimb(Drive driveSubsystem, Climb climbSubsystem) {
        this.climb = climbSubsystem;
        this.drive = driveSubsystem;

        sequence = new SequentialCommandGroup(
                new ParallelCommandGroup(
                        drive.pathFromPose(ClimbUtils.getPreClimbTarget(drive)),
                        new Climb_move(climb, ClimbPosition.UP)),
                drive.pathFromPath(drive.pathFromPoseWithConstraints(ClimbUtils.getFinalClimbTarget(drive),
                        new Path.PathConstraints().setMaxVelocityMetersPerSec(0.5))),
                new Climb_move(climb, ClimbPosition.DOWN));

        addRequirements(climbSubsystem, driveSubsystem);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        sequence.initialize();
        
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        sequence.execute();
    }

    @Override
    public boolean isFinished() {
        return sequence.isFinished();
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        sequence.cancel();
        climb.setClimbSpeed(0);
    }

}
