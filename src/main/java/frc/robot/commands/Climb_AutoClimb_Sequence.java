package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.climb.Climb;
import frc.robot.subsystems.climb.Climb.ClimbPosition;
import frc.robot.subsystems.drive.Drive;
import frc.robot.util.driveUtils.ClimbUtils;

public class Climb_AutoClimb_Sequence {
        public static Command get(Drive drive, Climb climb) {
                return new SequentialCommandGroup(
                                new ParallelCommandGroup(
                                                drive.commandFromPath(ClimbUtils.getPreClimbTarget(drive)),
                                                new Climb_Move(climb, ClimbPosition.UP)),
                                drive.commandFromPath(ClimbUtils.getFinalClimbTarget(drive)),
                                new Climb_Move(climb, ClimbPosition.DOWN));
        }
}
