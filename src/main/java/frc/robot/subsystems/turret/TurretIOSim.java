package frc.robot.subsystems.turret;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.Constants.TURRET_CONSTANTS;

public class TurretIOSim implements TurretIO {
    private double turretMotorPose = 0;
    private Rotation2d targetRotation = Rotation2d.kZero;
    private boolean isReachable = true;

    public TurretIOSim() {
        turretMotorPose = 0;
    }

    @Override
    public void updateInputs(TurretIOInputs inputs) {
        inputs.robotRelativeRotation = getRobotRelativeRotation();
        inputs.turretRotation = inputs.robotRelativeRotation.getRotations();
        inputs.absTurretRotation = getRingRotation();
        inputs.targetRotation = targetRotation;
        inputs.canMakeItToTarget = isReachable;
        inputs.limitSwitch = (turretMotorPose <= 0.05);
    }

    @Override
    public void setTurretSetpoint(Rotation2d robotRelativeRotation) {
        this.targetRotation = robotRelativeRotation;
        double diffDeg = MathUtil.inputModulus(
            robotRelativeRotation.getDegrees() - TURRET_CONSTANTS.LIMIT_SWITCH_ROBOT_ROTATION.getDegrees(),
            0.0, 360.0
        );
        double mechRotations = diffDeg / 360.0;
        if (mechRotations <= TURRET_CONSTANTS.ROTATION_RANGE_IN_ROT) {
            isReachable = true;
            turretMotorPose = mechRotations * TURRET_CONSTANTS.MOTOR_TO_RING_RATIO;
        } else {
            isReachable = false;
            double distToMax = mechRotations - TURRET_CONSTANTS.ROTATION_RANGE_IN_ROT;
            double distToMin = 1.0 - mechRotations;
            if (distToMax < distToMin) {
                turretMotorPose = TURRET_CONSTANTS.ROTATION_RANGE_IN_ROT * TURRET_CONSTANTS.MOTOR_TO_RING_RATIO;
            } else {
                turretMotorPose = 0.0;
            }
        }
    }

    @Override
    public void setTurretSetpoint(double rotations) {
        setTurretSetpoint(Rotation2d.fromRotations(rotations));
    }

    @Override
    public void setTurretZero() {
        turretMotorPose = 0;
    }

    @Override
    public double getRingRotation() {
        return turretMotorPose / TURRET_CONSTANTS.MOTOR_TO_RING_RATIO;
    }

    @Override
    public Rotation2d getRobotRelativeRotation() {
        double mechRotations = getRingRotation();
        return TURRET_CONSTANTS.LIMIT_SWITCH_ROBOT_ROTATION.plus(Rotation2d.fromRotations(mechRotations));
    }

    @Override
    public double getRobotRelitiveRotation() {
        return getRobotRelativeRotation().getRotations();
    }
}
