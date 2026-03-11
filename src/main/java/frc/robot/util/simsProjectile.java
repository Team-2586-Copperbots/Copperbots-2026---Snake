package frc.robot.util;

import static edu.wpi.first.units.Units.MetersPerSecond;

import java.util.function.Supplier;

import org.ironmaple.simulation.seasonspecific.rebuilt2026.RebuiltFuelOnFly;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.LinearVelocity;
import frc.robot.Constants;
import frc.robot.Constants.SHOOTER_CONSTANTS;

public class simsProjectile {
        Supplier<Pose2d> robotPose;
        Supplier<ChassisSpeeds> fieldRelativeSpeeds;
        Supplier<Rotation2d> turretRotation;
        Supplier<Double> shooterSpeed;

        public simsProjectile(Supplier<Pose2d> robotPose, Supplier<ChassisSpeeds> fieldRelativeSpeeds,
                        Supplier<Rotation2d> turretRotation, Supplier<Double> shooterSpeed) {
                this.robotPose = robotPose;
                this.fieldRelativeSpeeds = fieldRelativeSpeeds;
                this.turretRotation = turretRotation;
                this.shooterSpeed = shooterSpeed;

        }

        public void shootLemmon() {
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
                                                * Constants.SHOOTER_CONSTANTS.SHOOTER_WHEELE_CIRCUMFERENCE / 2,
                                                MetersPerSecond),
                                // The angle at which the note is launched
                                SHOOTER_CONSTANTS.SHOOTER_HOOD_ANGLE); 
        }
}
