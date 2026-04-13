package frc.robot.util.driveUtils;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.units.measure.Distance;
import frc.robot.lib.BLine.Path;
import frc.robot.lib.BLine.Path.PathConstraints;
import frc.robot.subsystems.drive.BLine_Constants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.util.AllianceFlipUtil;

public class BrutalClimbUtils {

    private static final PathConstraints finalConstraints = BLine_Constants.highTolerence
            .setMaxVelocityMetersPerSec(0.25);
    /*
     * Changables
     */

    // TODO: update poses when at comp/home and compare
    private static Pose2d FINAL_CLIMB_TARGET_UPPER = new Pose2d(0.982, 4.497326, new Rotation2d(Degrees.of(90)));
    private static Pose2d FINAL_CLIMB_TARGET_LOWER = new Pose2d(0.982, 2.955326, new Rotation2d(Degrees.of(-90)));

    private static Pose2d FINAL_CLIMB_TARGET = new Pose2d();
    private static Pose2d PRE_CLIMB_TARGET = new Pose2d();

    // climb target specific math
    private static Distance climbSideFlipingDistanceFromBottom = Meters.of(3.9747);

    // true for top climb, false for bottom climb
    private static boolean getIsTopClimb() {
        // upper climb
        Logger.recordOutput("ClimbUtils/distance to flip at", climbSideFlipingDistanceFromBottom.in(Meters));
        Logger.recordOutput("ClimbUtils/fliped pose Y", AllianceFlipUtil.applyY(Drive.getInstance().getPose().getY()));
        Logger.recordOutput("ClimbUtils/should be top", (climbSideFlipingDistanceFromBottom.in(Meters))
                - AllianceFlipUtil.applyY(Drive.getInstance().getPose().getY()) > 0);

        if ((climbSideFlipingDistanceFromBottom.in(Meters))
                - AllianceFlipUtil.applyY(Drive.getInstance().getPose().getY()) > 0) {
            return true;
        } else {
            return false;
        }
    }

    public static Path getFinalClimbTarget(Drive drive) {
        // this method ajusts the final target for the offset of the climb
        if (getIsTopClimb()) {
            // upper target
            FINAL_CLIMB_TARGET = FINAL_CLIMB_TARGET_UPPER;
        } else {
            // lower target
            FINAL_CLIMB_TARGET = FINAL_CLIMB_TARGET_LOWER;
        }
        return drive.pathFromPoseWithConstraints(FINAL_CLIMB_TARGET, finalConstraints);
    }

    public static Path getPreClimbTarget(Drive drive) {
        // this method moves the target out so the drive can to the climb in a straight
        // line
        getFinalClimbTarget(drive);

        double amountOut = 1;

        if (getIsTopClimb()) {
            PRE_CLIMB_TARGET = FINAL_CLIMB_TARGET
                    .plus(new Transform2d(Meters.of(amountOut), Meters.of(0), FINAL_CLIMB_TARGET.getRotation()));
        } else {
            PRE_CLIMB_TARGET = FINAL_CLIMB_TARGET
                    .plus(new Transform2d(Meters.of(-amountOut), Meters.of(0), FINAL_CLIMB_TARGET.getRotation()));
        }

        return drive.pathFromPoseWithConstraints(PRE_CLIMB_TARGET, BLine_Constants.highTolerence);

    }

}
