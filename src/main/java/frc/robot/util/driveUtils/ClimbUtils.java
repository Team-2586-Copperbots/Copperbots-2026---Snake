package frc.robot.util.driveUtils;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Rotations;

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
        if ((climbSideFlipingDistanceFromBottom.in(Meters))
                - AllianceFlipUtil.applyY(drive.getPose().getY()) > 0) {
            return true;
        } else {
            return false;
        }
    }

    public static Pose2d getFinalClimbTarget(Drive drive) {
        // this method ajusts the final target for the offset of the climb
        Distance climbOffsetFromRobotCenter = Inches.of(3.25);
        if (getIsTopClimb(drive)) {
            // upper target
            FINAL_CLIMB_TARGET = new Pose2d(Meters.of(1.01).plus(climbOffsetFromRobotCenter),
                            Meters.of(4.35), new Rotation2d(Degrees.of(90)));
        } else {
            // lower target
            FINAL_CLIMB_TARGET = new Pose2d(Meters.of(1.01).minus(climbOffsetFromRobotCenter),
                            Meters.of(3.14), new Rotation2d(Degrees.of(-90)));
        }
        return FINAL_CLIMB_TARGET;
    }

    public static Pose2d getPreClimbTarget(Drive drive) {
        // this method moves the target out so the drive can to the climb in a straight
        // line
        Pose2d finalTarget = getFinalClimbTarget(drive);
        Distance amountOut = Meters.of(1);
        if (getIsTopClimb(drive)) {
            PRE_CLIMB_TARGET = new Pose2d(finalTarget.getMeasureX(),
                    finalTarget.getMeasureY().plus(amountOut), finalTarget.getRotation());
        } else {
            PRE_CLIMB_TARGET = new Pose2d(finalTarget.getMeasureX(),
                    finalTarget.getMeasureY().minus(amountOut), finalTarget.getRotation());
        }

        return PRE_CLIMB_TARGET;

    }

    
}
