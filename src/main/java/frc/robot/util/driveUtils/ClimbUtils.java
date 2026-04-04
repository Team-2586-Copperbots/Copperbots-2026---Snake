package frc.robot.util.driveUtils;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Rotations;

import org.littletonrobotics.junction.AutoLogOutput;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Distance;
import frc.robot.lib.BLine.Path;
import frc.robot.lib.BLine.Path.PathConstraints;
import frc.robot.subsystems.drive.BLine_Constants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.util.AllianceFlipUtil;

public class ClimbUtils {

    public static final PathConstraints finalConstraings = BLine_Constants.highTolerence.setMaxVelocityMetersPerSec(0.25);

    private static Pose2d FINAL_CLIMB_TARGET = new Pose2d();
    private static Pose2d PRE_CLIMB_TARGET = new Pose2d();

    // climb target specific math
    private static Distance climbSideFlipingDistanceFromBottom = Meters.of(3.9747);

    @AutoLogOutput
    // true for top climb, false for bottom climb
    private static boolean getIsTopClimb() {
        // upper climb
        if ((climbSideFlipingDistanceFromBottom.in(Meters))
                - AllianceFlipUtil.applyY(Drive.getInstance().getPose().getY()) > 0) {
            return true;
        } else {
            return false;
        }
    }

    public static Path getFinalClimbTarget(Drive drive) {
        // this method ajusts the final target for the offset of the climb
        Distance climbOffsetFromRobotCenter = Inches.of(3.75);
        if (getIsTopClimb()) {
            // upper target
            FINAL_CLIMB_TARGET = new Pose2d(Meters.of(1.01).plus(climbOffsetFromRobotCenter),
                            Meters.of(4.37), new Rotation2d(Degrees.of(90)));
        } else {
            // lower target
            FINAL_CLIMB_TARGET = new Pose2d(Meters.of(1.01).minus(climbOffsetFromRobotCenter),
                            Meters.of(2.89), new Rotation2d(Degrees.of(-90)));
        }
        return drive.pathFromPoseWithConstraints(FINAL_CLIMB_TARGET, finalConstraings);
    }

    public static Path getPreClimbTarget(Drive drive) {
        // this method moves the target out so the drive can to the climb in a straight
        // line
        getFinalClimbTarget(drive);

        Distance amountOut = Meters.of(0.5);

        if (getIsTopClimb()) {
            PRE_CLIMB_TARGET = new Pose2d(FINAL_CLIMB_TARGET.getMeasureX(),
                    FINAL_CLIMB_TARGET.getMeasureY().plus(amountOut), FINAL_CLIMB_TARGET.getRotation());
        } else {
            PRE_CLIMB_TARGET = new Pose2d(FINAL_CLIMB_TARGET.getMeasureX(),
                    FINAL_CLIMB_TARGET.getMeasureY().minus(amountOut), FINAL_CLIMB_TARGET.getRotation());
        }

        return drive.pathFromPoseWithConstraints(PRE_CLIMB_TARGET, BLine_Constants.highTolerence);

    }

    
}
