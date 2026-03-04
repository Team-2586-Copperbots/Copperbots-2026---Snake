package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.PLACES;
import frc.robot.Constants.SHOOTER_CONSTANTS;
import frc.robot.Constants.TURRET_CONSTANTS;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.drive.Drive;

public final class Utils {

    public static boolean isAllianceBlue() {
        var alliance = DriverStation.getAlliance();
        if (alliance.isPresent()) {
            return alliance.get() == DriverStation.Alliance.Blue;
        }
        return false;
    }

    public static double shooterSpeedFromDistance(double distance) {
        // regresion equation for shooter
        double speed = ((4.86 * distance) + 35.7);

        return speed;
    }

    public static double timeFromDistance(double distance) {
        // TODO: make vagly accurate
        double time = (0.25 * distance);
        return time;
    }

    public static double distanceFromPose(Pose2d taretPose2d, Drive drivetrain) {

        // double distance = taretPose3d.getTranslation().
        Pose2d shooterPose2d = pose2dForShooter(drivetrain);

        double distanceX = Math.abs(taretPose2d.getX() - shooterPose2d.getX());
        double distanceY = Math.abs(taretPose2d.getY() - shooterPose2d.getY());
        double distanceXY = Math.sqrt((Math.pow(distanceX, 2) + Math.pow(distanceY, 2)));
        return distanceXY;
    }

    public static Pose2d pose2dForShooter(Drive drivetrain) {
        // done: update with math for the pose of the shooter from the cad modle do
        // using (unit circle and angle (in radians)) or wario

        Pose2d robotPose2d = drivetrain.getPose();

        double shooterXOffset = (Math.cos(robotPose2d.getRotation().getRadians())
                * TURRET_CONSTANTS.TURRET_OFFSET_FROM_ROBOT_CENTER.getX());

        double shooterYOffset = (Math.sin(robotPose2d.getRotation().getRadians())
                * TURRET_CONSTANTS.TURRET_OFFSET_FROM_ROBOT_CENTER.getY());

        Pose2d shooterPose2d = new Pose2d(robotPose2d.getX() + shooterXOffset, robotPose2d.getY() + shooterYOffset,
                robotPose2d.getRotation());

        return shooterPose2d;
    }

    // this returns the angle (in rot) fron the center of the robot to the center of
    // the hubs
    // schoeing element by way of math and arcsin()
    public static double getAngleToHub(Drive drivetrain) {
        Pose2d shooterPose2d = pose2dForShooter(drivetrain);

        double x = Math.abs(shooterPose2d.getX() - PLACES.CENTER_OF_HUB.getX());
        double y = Math.abs(shooterPose2d.getY() - PLACES.CENTER_OF_HUB.getY());

        // in rotations
        double baseTurretAngle = Units.radiansToRotations(Math.atan(y / x));

        if (shooterPose2d.getY() > PLACES.CENTER_OF_HUB.getY()) {
            baseTurretAngle = -baseTurretAngle;
        }

        // factor in drivetrain rotation
        double angle = (baseTurretAngle + drivetrain.getPose().getRotation().getRotations()) % 1;

        return angle;
    }

    public static double getAngleToHubWithVelocity(Drive drivetrain) {
        double angle = 0;

        double robotX = drivetrain.getPose().getX();
        double robotY = drivetrain.getPose().getY();

        double velocityX = drivetrain.getChassisSpeeds().vxMetersPerSecond;
        double velocityY = drivetrain.getChassisSpeeds().vyMetersPerSecond;

        double seconds = timeFromDistance(distanceFromPose(PLACES.CENTER_OF_HUB, drivetrain));

        double velocityXDistance = (velocityX * seconds);
        double velocityYDistance = (velocityY * seconds);

        double xTotal = (robotX + velocityXDistance);
        double yTotal = (robotY + velocityYDistance);

        double xAim = xTotal - PLACES.CENTER_OF_HUB.getX();
        double yAim = yTotal - PLACES.CENTER_OF_HUB.getY();
        angle = Units.radiansToRotations(Math.atan(yAim / xAim));

        // factor in drivetrain rotation
        angle = -angle + -drivetrain.getPose().getRotation().getRotations();

        return angle;
    }

}
