package frc.robot.subsystems.turret;

import frc.robot.Constants.TURRET_CONSTANTS;

public class TurretIOSim implements TurretIO {
    private double turretMotorPose;
    private double motorSpeed = 0;
    private boolean isClosedLoop = true;
    private boolean isAtPosition = true;

    public TurretIOSim() {
        turretMotorPose = 0;
    }

    @Override
    public void updateInputs(TurretIOInputs inputs) {
        inputs.isClosedLoop = isClosedLoop;
        inputs.currentRingPose = turretMotorPose;
        inputs.currentRingSpeed = motorSpeed;
        inputs.limitSwitch = false;
        inputs.rotationRelitiveToRobotZero = getRobotRelitiveRotation();
        inputs.isAtPosition = isAtPosition;
    }

    @Override
    public void setTurretSetpoint(double roations) {
        isClosedLoop = true;

        // limits are typed in as degres
        if (roations >= (0 - TURRET_CONSTANTS.TURRET_RING_ZERO_TO_ROBOT_BACK_OFFSET)
                && roations < TURRET_CONSTANTS.ROTATION_RANGE_IN_ROT) {
            isAtPosition = true;
            turretMotorPose = ((roations + TURRET_CONSTANTS.TURRET_RING_ZERO_TO_ROBOT_BACK_OFFSET)
                    * TURRET_CONSTANTS.MOTOR_TO_RING_RATIO);
        } else {
            // says that it did not make it to the desired position
            isAtPosition = false;

            // goes to nearist point relative to desired point
            double startOfDeadZone = 0;
            double endOfDeadZone = TURRET_CONSTANTS.ROTATION_RANGE_IN_ROT;

            if (Math.abs((startOfDeadZone - roations)) < Math.abs((endOfDeadZone - roations))) {
                turretMotorPose = (0 * TURRET_CONSTANTS.MOTOR_TO_RING_RATIO);
            } else if (Math.abs((startOfDeadZone - roations)) >= Math.abs((endOfDeadZone - roations))) {
                turretMotorPose = (TURRET_CONSTANTS.ROTATION_RANGE_IN_ROT * TURRET_CONSTANTS.MOTOR_TO_RING_RATIO);
            } else {
                turretMotorPose = (0 * TURRET_CONSTANTS.MOTOR_TO_RING_RATIO);
            }

        }
    }

    // set the turn motors's internal encoder
    @Override
    public void setTurretZero() {
        turretMotorPose = 0;
    }

    @Override
    public double getRingRotation() {
        return (turretMotorPose / TURRET_CONSTANTS.MOTOR_TO_RING_RATIO);
    }

    @Override
    public double getRobotRelitiveRotation() {
        return -0.5 + (turretMotorPose / TURRET_CONSTANTS.MOTOR_TO_RING_RATIO)
                + TURRET_CONSTANTS.TURRET_RING_ZERO_TO_ROBOT_BACK_OFFSET;
    }

}
