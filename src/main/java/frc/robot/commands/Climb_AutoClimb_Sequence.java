package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.lib.BLine.Path;
import frc.robot.subsystems.climb.Climb;
import frc.robot.subsystems.climb.Climb.ClimbPosition;
import frc.robot.subsystems.drive.Drive;
import frc.robot.util.driveUtils.ClimbUtils;

public class Climb_AutoClimb_Sequence {
        public static Command get(Drive drive, Climb climb) {
                return new SequentialCommandGroup(
                                new ParallelCommandGroup(
                                                drive.pathFromPose(ClimbUtils.getPreClimbTarget(drive)),
                                                new Climb_move(climb, ClimbPosition.UP)),
                                drive.pathFromPath(
                                                drive.pathFromPoseWithConstraints(ClimbUtils.getFinalClimbTarget(drive),
                                                                new Path.PathConstraints()
                                                                                .setMaxVelocityMetersPerSec(0.5))),
                                new Climb_move(climb, ClimbPosition.DOWN));
        }
}
