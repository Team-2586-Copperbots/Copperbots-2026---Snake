package frc.robot.subsystems.turret;

import static edu.wpi.first.units.Units.Rotations;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Angle;
import frc.robot.Constants.TURRET_CONSTANTS;

public class TurretIOSim implements TurretIO {
    private Rotation2d turretMotorPose;
    private double motorSpeed = 0;
    private boolean isClosedLoop = true;
    private boolean isAtPosition = true;

    public TurretIOSim() {
        turretMotorPose = Rotation2d.k180deg;
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
    public void setTurretSetpoint(Rotation2d roations) {
        isClosedLoop = true;

        // limits are typed in as degres
        if (roations.getRotations() >= (-TURRET_CONSTANTS.TURRET_RING_MINIMUM_TO_ROBOT_BACK_OFFSET.getRotations())
                && TURRET_CONSTANTS.ROTATION_RANGE_IN_ROT.minus(roations).getRotations() < 0) {
            isAtPosition = true;
            turretMotorPose = new Rotation2d( Angle.ofBaseUnits(((roations.plus(TURRET_CONSTANTS.TURRET_RING_MINIMUM_TO_ROBOT_BACK_OFFSET).getRotations())
                    * TURRET_CONSTANTS.MOTOR_TO_RING_RATIO), Rotations));
        } else {
            // says that it did not make it to the desired position
            isAtPosition = false;

            // goes to nearist point relative to desired point
            Rotation2d minOfDeadZone = Rotation2d.kZero;
            Rotation2d maxOfDeadZone = TURRET_CONSTANTS.ROTATION_RANGE_IN_ROT;

            if (Math.abs((minOfDeadZone.minus(roations).getRotations())) < Math.abs((maxOfDeadZone.minus(roations).getRotations()))) {
                turretMotorPose = new Rotation2d(Angle.ofBaseUnits((Rotation2d.kZero.getRotations() * TURRET_CONSTANTS.MOTOR_TO_RING_RATIO), Rotations));
            } else if (Math.abs((minOfDeadZone.minus(roations).getRotations())) >= Math.abs((maxOfDeadZone.minus(roations).getRotations()))) {
                turretMotorPose = new Rotation2d(Angle.ofBaseUnits((TURRET_CONSTANTS.ROTATION_RANGE_IN_ROT.getRotations() * TURRET_CONSTANTS.MOTOR_TO_RING_RATIO), Rotations));
            } else {
                turretMotorPose = new Rotation2d(Angle.ofBaseUnits((Rotation2d.kZero.getRotations() * TURRET_CONSTANTS.MOTOR_TO_RING_RATIO), Rotations));
            }

        }
    }

    // set the turn motors's internal encoder
    @Override
    public void setTurretZero() {
        turretMotorPose = Rotation2d.kZero;
    }

    @Override
    public Rotation2d getRingRotation() {
        return new Rotation2d(Angle.ofBaseUnits((turretMotorPose.getRotations() / TURRET_CONSTANTS.MOTOR_TO_RING_RATIO), Rotations));
    }

    @Override
    public Rotation2d getRobotRelitiveRotation() {
        return new Rotation2d(Angle.ofBaseUnits((turretMotorPose.getRotations() / TURRET_CONSTANTS.MOTOR_TO_RING_RATIO)
                + TURRET_CONSTANTS.TURRET_RING_MINIMUM_TO_ROBOT_BACK_OFFSET.getRotations(), Rotations)) ;
    }

}
