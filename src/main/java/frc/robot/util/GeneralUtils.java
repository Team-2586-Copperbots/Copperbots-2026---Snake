package frc.robot.util;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Rotations;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.FIELD_CONSTANTS;
import frc.robot.Constants.TURRET_CONSTANTS;
import frc.robot.subsystems.drive.Drive;

public final class GeneralUtils {

    public static boolean isAllianceBlue() {
        var alliance = DriverStation.getAlliance();
        if (alliance.isPresent()) {
            return alliance.get() == DriverStation.Alliance.Blue;
        }
        return false;
    }

    public static Pose2d findTarget(Drive drive) {
        if (drive.getPose().getX() > FIELD_CONSTANTS.CENTER_OF_HUB.getX()) {
            if (drive.getPose().getY() > FIELD_CONSTANTS.CENTER_OF_HUB.getY()) {
                return FIELD_CONSTANTS.TOP_FULE_STORAGE;
            } else {
                return FIELD_CONSTANTS.BOTTOM_FULE_STORAGE;
            }
        } else {
            return FIELD_CONSTANTS.CENTER_OF_HUB;
        }
    }

    public static double shooterSpeedFromDistance(double distance) {
        // regresion equation for shooter
        double speed = ((4.42 * distance) + 34.3);

        return speed;
    }

    public static double timeFromDistance(double distance) {
        // TODO: make vagly accurate
        double time = 0;
        // time = (0.25 * distance);
        return time;
    }

    public static double distanceFromPose(Pose2d taretPose2d, Drive drivetrain) {

        // double distance = taretPose3d.getTranslation().
        Pose2d shooterPose2d = pose2dForTurret(drivetrain);

        double distanceX = Math.abs(taretPose2d.getX() - shooterPose2d.getX());
        double distanceY = Math.abs(taretPose2d.getY() - shooterPose2d.getY());
        double distanceXY = Math.sqrt((Math.pow(distanceX, 2) + Math.pow(distanceY, 2)));
        return distanceXY;
    }

    public static Pose2d pose2dForTurret(Drive drivetrain) {
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
    public static Rotation2d getAngleToHubWithVelocity(Drive drivetrain) {
        double angle = 0;

        double robotX = drivetrain.getPose().getX();
        double robotY = drivetrain.getPose().getY();

        double velocityX = drivetrain.getChassisSpeeds().vxMetersPerSecond;
        double velocityY = drivetrain.getChassisSpeeds().vyMetersPerSecond;

        double seconds = timeFromDistance(distanceFromPose(FIELD_CONSTANTS.CENTER_OF_HUB, drivetrain));

        double velocityXDistance = (velocityX * seconds);
        double velocityYDistance = (velocityY * seconds);

        double xTotal = (robotX + velocityXDistance);
        double yTotal = (robotY + velocityYDistance);

        double xAim = (FIELD_CONSTANTS.CENTER_OF_HUB.getX() + -xTotal);
        double yAim = (FIELD_CONSTANTS.CENTER_OF_HUB.getY() + -yTotal);
        SmartDashboard.putNumber("xAim", xAim);
        SmartDashboard.putNumber("yAim", yAim);
        angle = Units.radiansToRotations(Math.atan(yAim / xAim));
        SmartDashboard.putNumber("angle", angle);

        // factor in drivetrain rotation
        angle = (-angle + drivetrain.getPose().getRotation().getRotations()) % 1;

        return new Rotation2d(Angle.ofBaseUnits(angle, Rotations));
    }

    public Rotation2d getAngleToHubNewMath(Drive drivetrain) {
        Rotation2d angle = Rotation2d.kZero;
        Translation2d target = FIELD_CONSTANTS.CENTER_OF_HUB.getTranslation();

        // math startes
        // robot's pose
        Pose2d turretPose = pose2dForTurret(drivetrain);
        // velocity in meters per second
        ChassisSpeeds velocity = drivetrain.getChassisSpeeds();
        // how many seconds it will take for the fule to fly
        double seconds = timeFromDistance(distanceFromPose(FIELD_CONSTANTS.CENTER_OF_HUB, drivetrain));
        // find the distance that I need to offset the robot by
        Translation2d velocityDistance = new Translation2d(
                Distance.ofBaseUnits(velocity.vxMetersPerSecond * seconds, Meters),
                Distance.ofBaseUnits(velocity.vyMetersPerSecond * seconds, Meters));
        // make a pose of where the robot will be when the fule is "scored" (assuming the robot continues to move) 
        Pose2d futureTurretPose = new Pose2d(turretPose.getTranslation().plus(velocityDistance), turretPose.getRotation());
        // find the turret to target translation
        // ex: x = 2.5 y = -0.2
        Translation2d turretToTargetTranslation = futureTurretPose.getTranslation().minus(target);
        // uses rotation2D's cartesian to polar system to get a angle from one pose to another
        Rotation2d aimingAngle = new Rotation2d(turretToTargetTranslation.getX(), turretToTargetTranslation.getY());
        // old things to just put numbers for testing, advantageKit is proably better for this, by now 
        SmartDashboard.putNumber("xAim", turretToTargetTranslation.getX());
        SmartDashboard.putNumber("yAim", turretToTargetTranslation.getY());
        SmartDashboard.putNumber("angle in degres", aimingAngle.getDegrees());

        // factor in drivetrain rotation
        angle = new Rotation2d(Angle.ofBaseUnits(((aimingAngle.getRotations() + -drivetrain.getPose().getRotation().getRotations()) % 1), Rotations));

        return angle;
    }

}
