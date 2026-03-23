package frc.robot.util;

import static edu.wpi.first.units.Units.Inch;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;

import java.util.function.Supplier;

import org.ironmaple.simulation.seasonspecific.rebuilt2026.RebuiltFuelOnFly;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import frc.robot.Constants;
import frc.robot.Constants.SHOOTER_CONSTANTS;

public class simsProjectile {
        private static Supplier<Pose2d> robotPose;
        private static Supplier<ChassisSpeeds> fieldRelativeSpeeds;
        private static Supplier<Rotation2d> turretRotation;
        private static Supplier<Double> shooterSpeed;

        public static void createSimsProjectile(Supplier<Pose2d> robotPose, Supplier<ChassisSpeeds> fieldRelativeSpeeds,
                        Supplier<Rotation2d> turretRotation, Supplier<Double> shooterSpeed) {
                simsProjectile.robotPose = robotPose;
                simsProjectile.fieldRelativeSpeeds = fieldRelativeSpeeds;
                simsProjectile.turretRotation = turretRotation;
                simsProjectile.shooterSpeed = shooterSpeed;

        }

        public static void shootLemmon() {
                RebuiltFuelOnFly fuelOnFly = new RebuiltFuelOnFly(
                                // Specify the position of the chassis when the note is launched
                                robotPose.get().getTranslation(),
                                // Specify the translation of the shooter from the robot center (in the
                                // shooter’s reference frame)
                                Constants.TURRET_CONSTANTS.TURRET_OFFSET_FROM_ROBOT_CENTER.getTranslation(),
                                // Specify the field-relative speed of the chassis, adding it to the initial
                                // velocity of the projectile
                                fieldRelativeSpeeds.get(),
                                // The shooter facing direction is the same as the robot’s facing direction
                                robotPose.get().getRotation().plus(
                                                // Add the shooter’s rotation
                                                turretRotation.get()),
                                // Initial height of the flying note
                                Constants.SHOOTER_CONSTANTS.HEIGHT_OF_WHEEL_OFF_GROUND,
                                // The launch speed is proportional to the RPS

                                LinearVelocity.ofBaseUnits(shooterSpeed.get()
                                                * Constants.SHOOTER_CONSTANTS.SHOOTER_WHEELE_CIRCUMFERENCE.in(Meters)
                                                / 2,
                                                MetersPerSecond),
                                // The angle at which the note is launched
                                SHOOTER_CONSTANTS.SHOOTER_HOOD_ANGLE);
                fuelOnFly.withTargetPosition(
                                () -> new Pose3d(Constants.FIELD_CONSTANTS.CENTER_OF_HUB).getTranslation())
                                .withTargetTolerance(new Translation3d(
                                                Distance.ofBaseUnits(47, Inch),
                                                Distance.ofBaseUnits(47, Inch),
                                                Distance.ofBaseUnits(.2, Meters)));
                fuelOnFly
                                // Configure callbacks to visualize the flight trajectory of the projectile
                                .withProjectileTrajectoryDisplayCallBack(
                                                // Callback for when the fuel will eventually hit the target (if
                                                // configured)
                                                (pose3ds) -> Logger.recordOutput(
                                                                "Flywheel/FuelProjectileSuccessfulShot",
                                                                pose3ds.toArray(Pose3d[]::new)),
                                                // Callback for when the fuel will eventually miss the target, or if no
                                                // target is configured
                                                (pose3ds) -> Logger.recordOutput(
                                                                "Flywheel/FuelProjectileUnsuccessfulShot",
                                                                pose3ds.toArray(Pose3d[]::new)));
        }
}
