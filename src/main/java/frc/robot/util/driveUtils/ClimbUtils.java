package frc.robot.util.driveUtils;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Distance;
import frc.robot.subsystems.drive.Drive;
import frc.robot.util.AllianceFlipUtil;

public class ClimbUtils {

    public static Pose2d FINAL_CLIMB_TARGET = new Pose2d();
    public static Pose2d PRE_CLIMB_TARGET = new Pose2d();

    // climb target specific math
    public static Distance climbSideFlipingDistanceFromBottom = Meters.of(3.9747);

    // true for top climb, false for bottom climb
    public static boolean getIsTopClimb(Drive drive) {
        // upper climb
        if (AllianceFlipUtil.applyY(climbSideFlipingDistanceFromBottom.in(Meters))
                - AllianceFlipUtil.applyY(drive.getPose().getY()) > 0) {
            return true;
        } else {
            return false;
        }
    }

    public static Pose2d getFinalClimbTarget(Drive drive) {
        // this method ajusts the final target for the offset of the climb
        boolean topClimbTrue = getIsTopClimb(drive);
        Distance climbOffsetFromRobotCenter = Inches.of(3.75);
        if (topClimbTrue) {
            // upper target
            FINAL_CLIMB_TARGET = AllianceFlipUtil
                    .apply(new Pose2d(Meters.of(1.01).plus(climbOffsetFromRobotCenter),
                            Meters.of(4.61), Rotation2d.kCCW_90deg));
        } else {
            // lower target
            FINAL_CLIMB_TARGET = AllianceFlipUtil
                    .apply(new Pose2d(Meters.of(1.01).minus(climbOffsetFromRobotCenter),
                            Meters.of(2.88), Rotation2d.kCW_90deg));
        }
        return FINAL_CLIMB_TARGET;
    }

    public static Pose2d getPreClimbTarget(Drive drive) {
        // this method moves the target out so the drive can to the climb in a straight
        // line
        Pose2d finalTarget = getFinalClimbTarget(drive);
        Distance amountOut = Meters.of(-0.5);
        if (getIsTopClimb(drive)) {
            PRE_CLIMB_TARGET = AllianceFlipUtil.apply(new Pose2d(finalTarget.getMeasureX().plus(amountOut),
                    finalTarget.getMeasureY(), finalTarget.getRotation()));
        } else {
            PRE_CLIMB_TARGET = AllianceFlipUtil.apply(new Pose2d(finalTarget.getMeasureX().minus(amountOut),
                    finalTarget.getMeasureY(), finalTarget.getRotation()));
        }

        return PRE_CLIMB_TARGET;

    }

    
}
