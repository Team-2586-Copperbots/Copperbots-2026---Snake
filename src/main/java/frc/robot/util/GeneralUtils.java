package frc.robot.util;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Radians;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.FIELD_CONSTANTS;
import frc.robot.Constants.OPERATOR_CONSTANTS;
import frc.robot.Constants.SHOOTER_CONSTANTS;
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

    public static double squareNumber(Double number) {
        boolean negative = number < 0;
        number = number * number;
        if (negative) {
            number = -number;
        }
        return number;
    }

    public static Pose2d findTarget() {
        // depending on the aliance, this metoh will flip the logic for greater/lesser
        // than x
        // all it does is change if it checks what direction to chech the robot is in
        // our zone
        Pose2d drive = Drive.getInstance().getPose();
        if (DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == Alliance.Red) {
            if (drive.getX() > FIELD_CONSTANTS.CENTER_OF_HUB.getX()) {
                return FIELD_CONSTANTS.CENTER_OF_HUB;
            } else if (drive.getY() < FIELD_CONSTANTS.CENTER_OF_HUB.getY()) {
                return FIELD_CONSTANTS.TOP_FULE_STORAGE;
            } else {
                return FIELD_CONSTANTS.BOTTOM_FULE_STORAGE;
            }
        } else {
            if (drive.getX() < FIELD_CONSTANTS.CENTER_OF_HUB.getX()) {
                return FIELD_CONSTANTS.CENTER_OF_HUB;
            } else if (drive.getY() > FIELD_CONSTANTS.CENTER_OF_HUB.getY()) {
                return FIELD_CONSTANTS.TOP_FULE_STORAGE;
            } else {
                return FIELD_CONSTANTS.BOTTOM_FULE_STORAGE;
            }
        }
    }

    public static double shooterSpeedFromTarget() {
        return shooterSpeedFromDistance(distanceFromTarget());
    }
    public static double shooterSpeedFromDistance(double distance) {
        // regresion equation for shooter
        Logger.recordOutput("Stuff/distancs for shooter", distance);
        return ((5.75 * distance) + 30.4);
    }

    public static double timeFromDistance(double distance) {
        // should be good
        double speed = shooterSpeedFromDistance(distance);
        double exitVelocity = (SHOOTER_CONSTANTS.SHOOTER_WHEELE_CIRCUMFERENCE.in(Meters) * speed) / 2;
        double xVelocity = Math.sin(SHOOTER_CONSTANTS.SHOOTER_HOOD_ANGLE.in(Radians)) * exitVelocity;
        double time = distance / (xVelocity);
        time += OPERATOR_CONSTANTS.LOOP_TIME;
        return time;
    }

    public static double distanceFromTarget() {
        return distanceFromTarget(findTarget());
    }

    public static double distanceFromTarget(Pose2d taretPose2d) {

        // double distance = taretPose3d.getTranslation().
        Pose2d shooterPose2d = translation2dForTurret();

        double distanceX = Math.abs(taretPose2d.getX() - shooterPose2d.getX());
        double distanceY = Math.abs(taretPose2d.getY() - shooterPose2d.getY());
        return Math.sqrt((Math.pow(distanceX, 2) + Math.pow(distanceY, 2)));
    }

    // field relitive speeds
    public static ChassisSpeeds getFieldRelitiveSpeeds() {

        // my math, outdated
        // // robot relitive speeds
        // ChassisSpeeds robotSpeeds = drivetrain.getChassisSpeeds();
        // Rotation2d robotHeading = drivetrain.getRotation().plus(/* chassie relitive
        // heading */new Rotation2d(
        // robotSpeeds.vxMetersPerSecond, robotSpeeds.vyMetersPerSecond));
        // LinearVelocity robotSpeed = MetersPerSecond
        // .of(Math.hypot(robotSpeeds.vxMetersPerSecond,
        // robotSpeeds.vyMetersPerSecond));
        // // use the field relitive heading of the robot and magnetude to
        // Translation2d fieldRelitiveSpeeds = new
        // Translation2d(robotSpeed.in(MetersPerSecond), robotHeading);

        // math using the swerve kinematics class
        ChassisSpeeds fieldRelitiveSpeeds = ChassisSpeeds.fromRobotRelativeSpeeds(
                Drive.getInstance().getChassisSpeeds(),
                Drive.getInstance().getRotation());
        Logger.recordOutput("Stuff/fieldrelitiveSpeeds/fieldRelitiveChassieSpeeds", fieldRelitiveSpeeds);
        Logger.recordOutput("Stuff/fieldrelitiveSpeeds/fieldrelitiveSpeedsPose",
                new Pose2d(fieldRelitiveSpeeds.vxMetersPerSecond, fieldRelitiveSpeeds.vyMetersPerSecond,
                        new Rotation2d(Radians.of(fieldRelitiveSpeeds.omegaRadiansPerSecond))));

        return fieldRelitiveSpeeds;
    }

    public static Pose2d translation2dForTurret() {
        // done: update with math for the pose of the shooter from the cad modle do
        // using (unit circle and angle (in radians)) or wario

        Pose2d robotPose2d = Drive.getInstance().getPose();

        double shooterXOffset = (Math.cos(robotPose2d.getRotation().getRadians())
                * TURRET_CONSTANTS.TURRET_OFFSET_FROM_ROBOT_CENTER.getX());

        double shooterYOffset = (Math.sin(robotPose2d.getRotation().getRadians())
                * TURRET_CONSTANTS.TURRET_OFFSET_FROM_ROBOT_CENTER.getY());

        return new Pose2d(robotPose2d.getX() + shooterXOffset, robotPose2d.getY() + shooterYOffset,
                robotPose2d.getRotation());
    }

    public static double getAngleToTarget() {
        return getAngleToTarget(findTarget());
    }

    public static double getAngleToTarget(Pose2d targetPose) {
        Translation2d target = new Translation2d(targetPose.getMeasureX(), targetPose.getMeasureY());

        // math startes
        // robot's pose
        Pose2d turretPose = translation2dForTurret();
        // velocity in meters per second
        ChassisSpeeds fieldVelocity = getFieldRelitiveSpeeds();
        // how many seconds it will take for the fule to fly
        double seconds = timeFromDistance(distanceFromTarget(targetPose));
        // find the distance that I need to offset the robot by
        Transform2d transformForVelcity = new Transform2d(
                Meters.of(fieldVelocity.vxMetersPerSecond * seconds),
                Meters.of(fieldVelocity.vyMetersPerSecond * seconds),
                new Rotation2d(Radians.of(fieldVelocity.omegaRadiansPerSecond * seconds)));

        // make a pose of where the robot will be when the fule is "scored" (assuming
        // the robot continues to move)
        Pose2d futureTurretPose = turretPose.plus(transformForVelcity);
        Logger.recordOutput("Stuff/futerTurretPose", futureTurretPose);
        // find the turret to target translation
        Translation2d turretToTargetTranslation = new Translation2d(target.getX() - futureTurretPose.getX(),
                futureTurretPose.getY() - target.getY());
        // uses rotation2D's cartesian to polar system to get a angle from one pose to
        // another
        // Rotation2d aimingAngle = new Rotation2d(turretToTargetTranslation.getX(),
        // turretToTargetTranslation.getY());
        double rotationAim = Units
                .radiansToRotations(Math.atan(turretToTargetTranslation.getY() / turretToTargetTranslation.getX()));

        // old things to just put numbers for testing, advantageKit is proably better
        // for this, by now

        // factor in drivetrain rotation
        rotationAim += AllianceFlipUtil.applyR(Drive.getInstance().getPose().getRotation()).getRotations();

        if (!targetPose.equals(FIELD_CONSTANTS.CENTER_OF_HUB)) {
            rotationAim += 0.5;
        }

        // for blue aliance
        if (SmartDashboard.getNumber("Polarity chooser", 1) == -1) {
            rotationAim += 0.5;
        }

        Logger.recordOutput("Stuff/rotationAim", rotationAim);

        // math to change numbers so they are within the range of the turret
        rotationAim = rotationAim % 1;
        if (rotationAim < (0 - TURRET_CONSTANTS.TURRET_RING_MINIMUM_TO_ROBOT_BACK_OFFSET)) {
            rotationAim += 1;
        }

        Logger.recordOutput("Stuff/rotationAim fixed for range", rotationAim);
        return rotationAim;
    }

}
