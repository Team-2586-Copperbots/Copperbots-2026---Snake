package frc.robot.util;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants.TURRET_CONSTANTS;
import org.junit.jupiter.api.Test;

public class GeneralUtilsTest {

    private static final double EPSILON = 1e-2;

    @Test
    public void testStationaryAimFacingTarget() {
        // Robot centered at (0, 0), facing directly towards the target at (5, 0)
        // With turret offset (-7.5", -8.5") in inches, turret is at (-0.1905, -0.2159)
        Pose2d robotPose = new Pose2d(0.0, 0.0, Rotation2d.kZero);
        ChassisSpeeds zeroSpeeds = new ChassisSpeeds(0.0, 0.0, 0.0);
        Pose2d target = new Pose2d(5.0, 0.0, Rotation2d.kZero);

        GeneralUtils.AimParameters aim = GeneralUtils.calculateAim(robotPose, zeroSpeeds, target);

        // Turret is slightly to the right (-Y), so aiming at (5, 0) requires a tiny positive (CCW / left) angle
        assertTrue(aim.isReachable());
        assertEquals(0.0, aim.robotRelativeAngle().getDegrees(), 3.0);
        assertTrue(aim.effectiveDistanceMeters() > 5.0);
    }

    @Test
    public void testStationaryAimRotatedChassis() {
        // Robot at (0, 0) rotated +90 degrees (facing +Y/North). Target is at (+5, 0) (East)
        // The turret must point towards East (+5, 0), which is -90 degrees relative to robot front!
        Pose2d robotPose = new Pose2d(0.0, 0.0, Rotation2d.fromDegrees(90.0));
        ChassisSpeeds zeroSpeeds = new ChassisSpeeds(0.0, 0.0, 0.0);
        Pose2d target = new Pose2d(5.0, 0.0, Rotation2d.kZero);

        GeneralUtils.AimParameters aim = GeneralUtils.calculateAim(robotPose, zeroSpeeds, target);

        assertTrue(aim.isReachable());
        // Robot-relative angle should be approximately -90 degrees
        assertEquals(-90.0, aim.robotRelativeAngle().getDegrees(), 4.0);
    }

    @Test
    public void testShootingOnTheMoveLeadStrafe() {
        // Robot at (0, 0) facing 0 degrees (East), strafing North (+Y) at 2.0 m/s
        // Target is directly East at (5.0, 0.0)
        // The ball inherits +Y velocity, so the virtual target must shift in -Y (South) to compensate!
        Pose2d robotPose = new Pose2d(0.0, 0.0, Rotation2d.kZero);
        ChassisSpeeds strafeSpeeds = new ChassisSpeeds(0.0, 2.0, 0.0);
        Pose2d target = new Pose2d(5.0, 0.0, Rotation2d.kZero);

        GeneralUtils.AimParameters aim = GeneralUtils.calculateAim(robotPose, strafeSpeeds, target);

        // Virtual target Y must be negative (shifted South to counteract Northward velocity)
        assertTrue(aim.virtualTarget().getY() < -0.1, "Virtual target must lead into strafe");
        // Aim angle must be negative (aiming South of East)
        assertTrue(aim.robotRelativeAngle().getDegrees() < 0.0, "Turret must aim into strafe");
    }

    @Test
    public void testShootingOnTheMoveLeadForward() {
        // Robot moving toward target (+X at 2.0 m/s). Target at (5.0, 0.0)
        // Ball inherits forward speed, so effective distance should be LESS than physical distance
        Pose2d robotPose = new Pose2d(0.0, 0.0, Rotation2d.kZero);
        ChassisSpeeds forwardSpeeds = new ChassisSpeeds(2.0, 0.0, 0.0);
        Pose2d target = new Pose2d(5.0, 0.0, Rotation2d.kZero);

        GeneralUtils.AimParameters aimMoving = GeneralUtils.calculateAim(robotPose, forwardSpeeds, target);
        GeneralUtils.AimParameters aimStill = GeneralUtils.calculateAim(robotPose, new ChassisSpeeds(), target);

        assertTrue(aimMoving.effectiveDistanceMeters() < aimStill.effectiveDistanceMeters(),
                "Driving forward should reduce effective shooting distance");
        assertTrue(aimMoving.shooterSpeedRPS() < aimStill.shooterSpeedRPS(),
                "Driving forward should reduce required flywheel speed");
    }

    @Test
    public void testTurretRangeReachableAndDeadzone() {
        // Limit switch is at ~195.84° (-164.16°). Range is 284.4°.
        // Reachable: -164.16° to +120.24° (covering front 0°, left +90°, right -90°)
        Pose2d robotPose = new Pose2d(0.0, 0.0, Rotation2d.kZero);
        ChassisSpeeds zeroSpeeds = new ChassisSpeeds();

        // Target in front (0°) -> reachable
        assertTrue(GeneralUtils.calculateAim(robotPose, zeroSpeeds, new Pose2d(5.0, 0.0, Rotation2d.kZero)).isReachable());

        // Target to left (+90°) -> reachable
        assertTrue(GeneralUtils.calculateAim(robotPose, zeroSpeeds, new Pose2d(0.0, 5.0, Rotation2d.kZero)).isReachable());

        // Target to right (-90°) -> reachable
        assertTrue(GeneralUtils.calculateAim(robotPose, zeroSpeeds, new Pose2d(0.0, -5.0, Rotation2d.kZero)).isReachable());

        // Target behind in deadzone (e.g., at 150° relative to robot)
        double angleRad = Units.degreesToRadians(150.0);
        Pose2d deadzoneTarget = new Pose2d(5.0 * Math.cos(angleRad), 5.0 * Math.sin(angleRad), Rotation2d.kZero);
        assertFalse(GeneralUtils.calculateAim(robotPose, zeroSpeeds, deadzoneTarget).isReachable(),
                "Target in rear deadzone should be marked unreachable");
    }
}

