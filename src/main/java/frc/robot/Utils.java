package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.Constants.PLACES;
import frc.robot.Constants.SHOOTER_CONSTANTS;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public final class Utils {

    public static boolean isAllianceBlue() {
        var alliance = DriverStation.getAlliance();
        if (alliance.isPresent()) {
            return alliance.get() == DriverStation.Alliance.Blue;
        }
        return false;
    }

    // TODO: when robot built, find if cole wants squared inputs
    public static double squareInput(double input) {
        boolean negative = input < 0;
        if (negative) {
            return -Math.pow(input, 2);
        } else {
            return Math.pow(input, 2);
        }
    }

    public static double shooterSpeedFromDistance(Pose3d taretPose3d, Pose3d shooterPose3d) {
        double speed = 0;
        // double distance = taretPose3d.getTranslation().
        double distanceX = Math.abs(taretPose3d.getX() - shooterPose3d.getX());
        double distanceY = Math.abs(taretPose3d.getY() - shooterPose3d.getY());
        double distanceZ = Math.abs(taretPose3d.getZ() - shooterPose3d.getZ());
        double distanceXY = Math.sqrt(Math.pow(distanceX, 2) + Math.pow(distanceY, 2));
        double distanceXYZ = Math.sqrt(Math.pow(distanceXY, 2) + Math.pow(distanceZ, 2));

        speed = ((5.45 * distanceXYZ) + 34.5);

        return speed;
    }

    public static Pose3d pose3dForShooter(CommandSwerveDrivetrain drivetrain) {
        Pose3d pose3d = new Pose3d(drivetrain.getState().Pose.getX(), drivetrain.getState().Pose.getY(),
                SHOOTER_CONSTANTS.HEIGHT_OF_WHEEL_OFF_GROUND, new Rotation3d(drivetrain.getState().Pose.getRotation()));

        return pose3d;
    }

    // this returns the angle (in rot) fron the center of the robot to the center of
    // the hubs
    // schoeing element by way of math and arcsin()
    public static double getAngleToHub(CommandSwerveDrivetrain drivetrain) {
        double angle = 0;

        double x = drivetrain.getState().Pose.getX() - PLACES.CENTER_OF_HUB.getX();
        double y = drivetrain.getState().Pose.getY() - PLACES.CENTER_OF_HUB.getY();
        angle = Units.radiansToRotations(Math.atan(y / x));

        // factor in drivetrain rotation
        angle = -angle + -drivetrain.getState().Pose.getRotation().getRotations();

        return angle;
    }

}
