package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.Constants.PLACES;
import frc.robot.Constants.SHOOTER_CONSTANTS;
import frc.robot.Constants.TURRET_CONSTANTS;
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

    public static double shooterSpeedFromDistance(double distance) {
        // regresion equation for shooter
        // TODO: update for mounted turret
        double speed = ((5.45 * distance) + 34.5);

        return speed;
    }

    public static double timeFromDistance(double distance) {
        // TODO: make vagly accurate
        double time = (0.25 * distance);
        return time;
    }

    public static double distanceFromPose(Pose3d taretPose3d, CommandSwerveDrivetrain drivetrain) {

        // double distance = taretPose3d.getTranslation().
        Pose3d shooterPose3d = pose3dForShooter(drivetrain);

        double distanceX = Math.abs(taretPose3d.getX() - shooterPose3d.getX());
        double distanceY = Math.abs(taretPose3d.getY() - shooterPose3d.getY());
        double distanceZ = Math.abs(taretPose3d.getZ() - shooterPose3d.getZ());
        double distanceXY = Math.sqrt(Math.pow(distanceX, 2) + Math.pow(distanceY, 2));
        double distanceXYZ = Math.sqrt(Math.pow(distanceXY, 2) + Math.pow(distanceZ, 2));

        return distanceXYZ;
    }

    public static Pose3d pose3dForShooter(CommandSwerveDrivetrain drivetrain) {
        // done: update with math for the pose of the shooter from the cad modle do
        // using (unit circle and angle (in radians)) or wario

        Pose2d robotPose2d = new Pose2d(drivetrain.getState().Pose.getX(), drivetrain.getState().Pose.getY(),
                drivetrain.getState().Pose.getRotation());

        double shooterXOffset = (Math.cos(robotPose2d.getRotation().getRadians())
                * TURRET_CONSTANTS.TURRET_OFFSET_FROM_ROBOT_CENTER.getX());

        double shooterYOffset = (Math.sin(robotPose2d.getRotation().getRadians())
                * TURRET_CONSTANTS.TURRET_OFFSET_FROM_ROBOT_CENTER.getY());

        Pose3d shooterPose3d = new Pose3d(robotPose2d.getX() + shooterXOffset, robotPose2d.getY() + shooterYOffset,
                SHOOTER_CONSTANTS.HEIGHT_OF_WHEEL_OFF_GROUND, new Rotation3d(robotPose2d.getRotation()));

        return shooterPose3d;
    }

    // this returns the angle (in rot) fron the center of the robot to the center of
    // the hubs
    // schoeing element by way of math and arcsin()
    public static double getAngleToHub(CommandSwerveDrivetrain drivetrain) {
        double angle = 0;
        Pose3d shooterPose3d = pose3dForShooter(drivetrain);

        double x = shooterPose3d.getX() - PLACES.CENTER_OF_HUB.getX();
        double y = shooterPose3d.getY() - PLACES.CENTER_OF_HUB.getY();
        
        // in rotations
        angle = Units.radiansToRotations(Math.atan(y / x));

        // factor in drivetrain rotation
        angle = -angle + -Units.radiansToRotations(shooterPose3d.getRotation().getAngle());

        return angle;
    }

    public static double getAngleToHubWithVelocity(CommandSwerveDrivetrain drivetrain) {
        double angle = 0;

        double robotX = drivetrain.getState().Pose.getX();
        double robotY = drivetrain.getState().Pose.getY();

        double velocityX = drivetrain.getState().Speeds.vxMetersPerSecond;
        double velocityY = drivetrain.getState().Speeds.vyMetersPerSecond;

        double seconds = timeFromDistance(distanceFromPose(PLACES.CENTER_OF_HUB, drivetrain));

        double velocityXDistance = (velocityX * seconds);
        double velocityYDistance = (velocityY * seconds);

        double xTotal = (robotX + velocityXDistance);
        double yTotal = (robotY + velocityYDistance);

        double xAim = xTotal - PLACES.CENTER_OF_HUB.getX();
        double yAim = yTotal - PLACES.CENTER_OF_HUB.getY();
        angle = Units.radiansToRotations(Math.atan(yAim / xAim));

        // factor in drivetrain rotation
        angle = -angle + -drivetrain.getState().Pose.getRotation().getRotations();

        return angle;
    }

}
