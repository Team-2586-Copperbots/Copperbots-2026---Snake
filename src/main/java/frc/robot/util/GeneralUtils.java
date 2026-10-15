package frc.robot.util;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Radians;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.Constants.FIELD_CONSTANTS;
import frc.robot.Constants.OPERATOR_CONSTANTS;
import frc.robot.Constants.SHOOTER_CONSTANTS;
import frc.robot.Constants.TURRET_CONSTANTS;
import frc.robot.subsystems.drive.Drive;

public final class GeneralUtils {

    /**
     * Record holding all computed aiming and shooting parameters.
     */
    public record AimParameters(
            Rotation2d robotRelativeAngle,
            Rotation2d fieldRelativeAngle,
            double effectiveDistanceMeters,
            double shooterSpeedRPS,
            double timeOfFlightSeconds,
            Translation2d virtualTarget,
            boolean isReachable) {
    }

    private GeneralUtils() {
    }

    public static boolean isAllianceBlue() {
        var alliance = DriverStation.getAlliance();
        return alliance.isPresent() && alliance.get() == DriverStation.Alliance.Blue;
    }

    public static double squareNumber(Double number) {
        if (number == null) return 0.0;
        return Math.copySign(number * number, number);
    }

    /**
     * Determines the active field target (Hub or back-field storage) based on robot zone.
     */
    public static Pose2d findTarget() {
        Pose2d drivePose = Drive.getInstance().getPose();
        boolean isRed = DriverStation.getAlliance().isPresent()
                && DriverStation.getAlliance().get() == Alliance.Red;

        if (isRed) {
            if (drivePose.getX() > FIELD_CONSTANTS.CENTER_OF_HUB.getX()) {
                return FIELD_CONSTANTS.CENTER_OF_HUB;
            } else if (drivePose.getY() < FIELD_CONSTANTS.CENTER_OF_HUB.getY()) {
                return FIELD_CONSTANTS.TOP_FULE_STORAGE;
            } else {
                return FIELD_CONSTANTS.BOTTOM_FULE_STORAGE;
            }
        } else {
            if (drivePose.getX() < FIELD_CONSTANTS.CENTER_OF_HUB.getX()) {
                return FIELD_CONSTANTS.CENTER_OF_HUB;
            } else if (drivePose.getY() > FIELD_CONSTANTS.CENTER_OF_HUB.getY()) {
                return FIELD_CONSTANTS.TOP_FULE_STORAGE;
            } else {
                return FIELD_CONSTANTS.BOTTOM_FULE_STORAGE;
            }
        }
    }

    /**
     * Calculates the field-relative 2D position of the turret center, accounting for
     * robot rotation and rigid-body mounting offset.
     */
    public static Translation2d getTurretFieldPosition(Pose2d robotPose) {
        return robotPose.getTranslation().plus(
                TURRET_CONSTANTS.TURRET_OFFSET_FROM_ROBOT_CENTER.rotateBy(robotPose.getRotation()));
    }

    /**
     * Backward-compatible helper returning a Pose2d representation of the turret.
     */
    public static Pose2d translation2dForTurret() {
        Pose2d robotPose = Drive.getInstance().getPose();
        return new Pose2d(getTurretFieldPosition(robotPose), robotPose.getRotation());
    }

    /**
     * Returns field-relative chassis speeds.
     */
    public static ChassisSpeeds getFieldRelitiveSpeeds() {
        return ChassisSpeeds.fromRobotRelativeSpeeds(
                Drive.getInstance().getChassisSpeeds(),
                Drive.getInstance().getRotation());
    }

    /**
     * Shooter speed regression equation from distance (in meters).
     */
    public static double shooterSpeedFromDistance(double distance) {
        Logger.recordOutput("Turret/Aim/DistanceForShooter", distance);
        return (5.98 * distance) + 29.8;
    }

    /**
     * Flight time calculation based on shooter exit velocity, hood angle, and loop delay.
     */
    public static double timeFromDistance(double distance) {
        double speed = shooterSpeedFromDistance(distance);
        double exitVelocity = (SHOOTER_CONSTANTS.SHOOTER_WHEELE_CIRCUMFERENCE.in(Meters) * speed) / 2.0;
        double xVelocity = Math.sin(SHOOTER_CONSTANTS.SHOOTER_HOOD_ANGLE.in(Radians)) * exitVelocity;
        if (xVelocity <= 1e-4) {
            return 0.5;
        }
        return (distance / xVelocity) + OPERATOR_CONSTANTS.LOOP_TIME;
    }

    /**
     * Calculates shooting on the move parameters for the current robot state and active target.
     */
    public static AimParameters calculateAim() {
        return calculateAim(Drive.getInstance().getPose(), getFieldRelitiveSpeeds(), findTarget());
    }

    /**
     * Core physics solver for shooting-on-the-move (SOTM) lead compensation.
     * Computes the virtual target offset to cancel robot momentum, required turret angle,
     * and distance-adjusted flywheel speed.
     */
    public static AimParameters calculateAim(Pose2d robotPose, ChassisSpeeds fieldVelocity, Pose2d targetPose) {
        Translation2d targetTranslation = targetPose.getTranslation();
        Translation2d turretPos = getTurretFieldPosition(robotPose);

        // 1. Initial distance & time of flight estimate
        double directDistance = turretPos.getDistance(targetTranslation);
        double timeOfFlight = timeFromDistance(directDistance);

        // 2. Iterative convergence to find the virtual target:
        //    P_virtual = P_target - (v_robot_field * t_flight)
        Translation2d virtualTarget = targetTranslation;
        double effectiveDistance = directDistance;
        for (int i = 0; i < 3; i++) { // TODO: why 3 iterations?
            Translation2d robotVelocityDisplacement = new Translation2d(
                    fieldVelocity.vxMetersPerSecond * timeOfFlight,
                    fieldVelocity.vyMetersPerSecond * timeOfFlight);
            virtualTarget = targetTranslation.minus(robotVelocityDisplacement);
            effectiveDistance = turretPos.getDistance(virtualTarget);
            timeOfFlight = timeFromDistance(effectiveDistance);
        }

        // 3. Field-relative angle pointing from turret to virtual target
        Translation2d turretToVirtualTarget = virtualTarget.minus(turretPos);
        Rotation2d fieldAimAngle = new Rotation2d(turretToVirtualTarget.getX(), turretToVirtualTarget.getY());

        // 4. Robot-relative angle: field aiming angle minus robot heading
        Rotation2d robotRelativeAngle = fieldAimAngle.minus(robotPose.getRotation());

        // 5. Soft limit and reachability check
        double diffDeg = MathUtil.inputModulus(
                robotRelativeAngle.getDegrees() - TURRET_CONSTANTS.LIMIT_SWITCH_ROBOT_ROTATION.getDegrees(),
                0.0, 360.0);
        boolean isReachable = (diffDeg / 360.0) <= TURRET_CONSTANTS.ROTATION_RANGE_IN_ROT;

        double shooterSpeed = shooterSpeedFromDistance(effectiveDistance);

        // 6. Telemetry logging for AdvantageKit / Glass
        Logger.recordOutput("Turret/Aim/TurretFieldPosition", new Pose2d(turretPos, fieldAimAngle));
        Logger.recordOutput("Turret/Aim/VirtualTarget", new Pose2d(virtualTarget, Rotation2d.kZero));
        Logger.recordOutput("Turret/Aim/FieldAimAngle", fieldAimAngle);
        Logger.recordOutput("Turret/Aim/RobotRelativeAngle", robotRelativeAngle);
        Logger.recordOutput("Turret/Aim/EffectiveDistance", effectiveDistance);
        Logger.recordOutput("Turret/Aim/ShooterRPS", shooterSpeed);
        Logger.recordOutput("Turret/Aim/IsReachable", isReachable);

        return new AimParameters(
                robotRelativeAngle,
                fieldAimAngle,
                effectiveDistance,
                shooterSpeed,
                timeOfFlight,
                virtualTarget,
                isReachable);
    }

    /**
     * Returns the required robot-relative turret angle to hit the target.
     */
    public static Rotation2d getAngleToTarget() {
        return calculateAim().robotRelativeAngle();
    }

    /**
     * Backward-compatible overload accepting a target pose.
     */
    public static Rotation2d getAngleToTarget(Pose2d targetPose) {
        return calculateAim(Drive.getInstance().getPose(), getFieldRelitiveSpeeds(), targetPose).robotRelativeAngle();
    }

    public static double shooterSpeedFromTarget() {
        return calculateAim().shooterSpeedRPS();
    }

    public static double distanceFromTarget() {
        return calculateAim().effectiveDistanceMeters();
    }

    public static double distanceFromTarget(Pose2d targetPose) {
        Translation2d turretPos = getTurretFieldPosition(Drive.getInstance().getPose());
        return turretPos.getDistance(targetPose.getTranslation());
    }
}
