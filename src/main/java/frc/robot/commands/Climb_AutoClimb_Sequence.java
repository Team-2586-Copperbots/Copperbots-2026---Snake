package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.climb.Climb;
import frc.robot.subsystems.climb.Climb.ClimbPosition;
import frc.robot.subsystems.drive.Drive;
import frc.robot.util.driveUtils.ManualClimbUtils;
import frc.robot.util.driveUtils.MathedClimbUtils;

public class Climb_AutoClimb_Sequence {
        // class to get a command sequence for automaticly climbing
        // does not work as of now, 4-8-2026
        // TODO: test and fix
        public static Command getMathed(Drive drive, Climb climb) {
                return new SequentialCommandGroup(
                                new ParallelCommandGroup(
                                                drive.commandFromPath(MathedClimbUtils.getPreClimbTarget(drive)),
                                                new Climb_move(climb, ClimbPosition.UP)),
                                drive.commandFromPath(MathedClimbUtils.getFinalClimbTarget(drive)),
                                new Climb_move(climb, ClimbPosition.DOWN));
        }

        public static Command getManual(Drive drive, Climb climb) {
                return new SequentialCommandGroup(
                                new ParallelCommandGroup(
                                                drive.commandFromPath(ManualClimbUtils.getPreClimbTarget(drive)),
                                                new Climb_move(climb, ClimbPosition.UP)),
                                drive.commandFromPath(ManualClimbUtils.getFinalClimbTarget(drive)),
                                new Climb_move(climb, ClimbPosition.DOWN));

        }

}
