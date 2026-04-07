package frc.robot.util.driveUtils;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.Distance;
import frc.robot.Constants.ROBOT_PROPERTIES;
import frc.robot.lib.BLine.Path;
import frc.robot.lib.BLine.Path.PathConstraints;
import frc.robot.subsystems.drive.BLine_Constants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.util.AllianceFlipUtil;

public class ClimbUtils {

    private static final PathConstraints finalConstraings = BLine_Constants.highTolerence
            .setMaxVelocityMetersPerSec(0.25);
    /*
     * Changables
     */
    private static final Distance climbWidth = Inches.of(35 + (3 / 8));
    private static final Distance uprightFaceWidth = Inches.of(3.51);
    private static final Distance lengthOfClimbFromWallX = Inches
            .of(43.75 - /* distance from front of upright to aliance wall */1.75);

    private static final Distance centerForClimbY = Inches
            .of(/* 0,0 to face of upright/base */130 + (climbWidth.in(Inches) / 2) - 6.25);

    public static final Translation2d centerOfClimbPose = new Translation2d(lengthOfClimbFromWallX, centerForClimbY);

    private static final Distance robotOffsetY = Inches
            .of((climbWidth.in(Inches) / 2) + ROBOT_PROPERTIES.lengthOffset /* tolererence off */ );
    private static final Distance robotOffsetX = Inches.of((uprightFaceWidth.in(Inches) / 2));

    private static Pose2d FINAL_CLIMB_TARGET = new Pose2d();
    private static Pose2d PRE_CLIMB_TARGET = new Pose2d();

    // climb target specific math
    private static Distance climbSideFlipingDistanceFromBottom = Meters.of(3.9747);

    
    // true for top climb, false for bottom climb
    private static boolean getIsTopClimb() {
        // upper climb
        Logger.recordOutput("a2", climbSideFlipingDistanceFromBottom.in(Meters));
        
        Logger.recordOutput("a1", AllianceFlipUtil.applyY(Drive.getInstance().getPose().getY()));
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
            FINAL_CLIMB_TARGET = new Pose2d(centerOfClimbPose.getMeasureX().plus(robotOffsetX),
                    centerOfClimbPose.getMeasureY().plus(robotOffsetY), new Rotation2d(Degrees.of(90)));
        } else {
            // lower target
            FINAL_CLIMB_TARGET = new Pose2d(centerOfClimbPose.getMeasureX().minus(robotOffsetX),
                    centerOfClimbPose.getMeasureY().minus(robotOffsetY), new Rotation2d(Degrees.of(-90)));
        }
        return drive.pathFromPoseWithConstraints(FINAL_CLIMB_TARGET, finalConstraings);
    }

    public static Path getPreClimbTarget(Drive drive) {
        // this method moves the target out so the drive can to the climb in a straight
        // line
        getFinalClimbTarget(drive);

        Distance amountOut = Meters.of(1);

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
