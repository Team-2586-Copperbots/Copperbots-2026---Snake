package frc.robot.commands;

import java.util.Set;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.climb.Climb;
import frc.robot.subsystems.climb.Climb.ClimbPosition;
import frc.robot.subsystems.drive.Drive;
import frc.robot.util.driveUtils.MathedClimbUtils;

public class Climb_AutoClimb_Sequence {
    // Generates an automated climbing sequence deferred to runtime so target poses
    // are calculated from live odometry rather than static robot-boot values.
    public static Command get(Drive drive, Climb climb) {
        return Commands.defer(() -> new SequentialCommandGroup(
                new ParallelCommandGroup(
                        drive.commandFromPath(MathedClimbUtils.getPreClimbTarget(drive)),
                        new Climb_Move(climb, ClimbPosition.UP)),
                drive.commandFromPath(MathedClimbUtils.getFinalClimbTarget(drive)),
                new Climb_Move(climb, ClimbPosition.DOWN)),
                Set.of(drive, climb));
    }
}
