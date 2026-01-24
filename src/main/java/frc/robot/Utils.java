package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
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

    public static double shooterSpeedFromDistance(Pose3d taretPose3d, Pose3d shooterPose3d) {
        double speed = 0;
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
                SHOOTER_CONSTANTS.HEIGHT_OF_WHEEL_OFF_GROUND, null);

        return pose3d;
    }

    // this returns the angle fron the center of the robot to the center of the hubs
    // schoeing element by way of math and arcsin()
    public static double getAngleToHub(CommandSwerveDrivetrain drivetrain) {
        double angle = 0;
        Pose2d drivetrainPose2d = drivetrain.getState().Pose;
        Pose3d hubPose3d = PLACES.CENTER_OF_HUB;
        if (hubPose3d.getX() - drivetrainPose2d.getX() > 0) {
            Pose2d relitiveHubPose2d = new Pose2d((drivetrainPose2d.getX() - hubPose3d.getX()),
                    (drivetrainPose2d.getY() - hubPose3d.getY()), null);

            angle = (Math.asin(Math.abs(relitiveHubPose2d.getY()) / Math.abs(relitiveHubPose2d.getX())) / Math.PI)
                    * 180;
            if (relitiveHubPose2d.getY() > 0) {
                angle = -angle;
            }
        }
        // degres to rotations
        angle = angle / 360;

        // factor in drivetrain rotation
        angle = angle + -drivetrainPose2d.getRotation().getRotations();
        return angle;
    }

}
