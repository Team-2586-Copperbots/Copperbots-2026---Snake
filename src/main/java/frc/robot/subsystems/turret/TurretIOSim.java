package frc.robot.subsystems.turret;

import frc.robot.Constants.TURRET_CONSTANTS;
import frc.robot.util.auto_logging_stuff.SimMotorAutoLogged;

public class TurretIOSim implements TurretIO {
    @SuppressWarnings("unused")
    private SimMotorAutoLogged turnMotor;
    private double turretMotorPose;
    @SuppressWarnings("unused")
    private double motorSpeed = 0;
    @SuppressWarnings("unused")
    private boolean isClosedLoop = true;
    @SuppressWarnings("unused")
    private boolean isAtPosition = true;

    public TurretIOSim() {
        turretMotorPose = 0;
    }

    @Override
    public void updateInputs(TurretIOInputs inputs) {

    }

    @Override
    public void setTurretSetpoint(double rotations) {
        isClosedLoop = true;

        // limits are typed in as degrees
        if (rotations >= (-TURRET_CONSTANTS.TURRET_RING_MINIMUM_TO_ROBOT_BACK_OFFSET)
                && (TURRET_CONSTANTS.ROTATION_RANGE_IN_ROT - rotations) < 0) {
            isAtPosition = true;
            turretMotorPose = (rotations + TURRET_CONSTANTS.TURRET_RING_MINIMUM_TO_ROBOT_BACK_OFFSET)
                    * TURRET_CONSTANTS.MOTOR_TO_RING_RATIO;
        } else {
            // says that it did not make it to the desired position
            isAtPosition = false;

        }
    }

    // set the turn motors's internal encoder
    @Override
    public void setTurretZero() {
        turretMotorPose = 0;
    }

    @Override
    public double getRingRotation() {
        return turretMotorPose / TURRET_CONSTANTS.MOTOR_TO_RING_RATIO;
    }

    @Override
    public double getRobotRelativeRotation() {
        return (turretMotorPose / TURRET_CONSTANTS.MOTOR_TO_RING_RATIO)
                + TURRET_CONSTANTS.TURRET_RING_MINIMUM_TO_ROBOT_BACK_OFFSET;
    }

}
