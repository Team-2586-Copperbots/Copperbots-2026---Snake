package frc.robot.subsystems.turret;

import frc.robot.Constants.TURRET_CONSTANTS;
import frc.robot.util.auto_logging_stuff.SimMotorAutoLogged;
import frc.robot.util.auto_logging_stuff.TalonFXInputsAutoLogged;

public class TurretIOSim implements TurretIO {
    private SimMotorAutoLogged turnMotor;
    private boolean canMakeItToTarget = true;

    public TurretIOSim() {
        turnMotor = new SimMotorAutoLogged();
    }

    @Override
    public void updateInputs(TurretIOInputs inputs) {

    }

    @Override 
    public TalonFXInputsAutoLogged getMotorInputs() {
        return turnMotor.getInputs();
    }

    @Override
    public void setTurretSetpoint(double roations) {
        // limits are typed in as degres
        if (roations >= (-TURRET_CONSTANTS.TURRET_RING_MINIMUM_TO_ROBOT_BACK_OFFSET)
                && (TURRET_CONSTANTS.ROTATION_RANGE_IN_ROT - roations) < 0) {
            canMakeItToTarget = true;
            turnMotor.setSimTarget(((roations + TURRET_CONSTANTS.TURRET_RING_MINIMUM_TO_ROBOT_BACK_OFFSET)
                    * TURRET_CONSTANTS.MOTOR_TO_RING_RATIO), false);
        } else {
            canMakeItToTarget = false;
            turnMotor.setSimTarget(
                    TURRET_CONSTANTS.TURRET_RING_MINIMUM_TO_ROBOT_BACK_OFFSET * TURRET_CONSTANTS.MOTOR_TO_RING_RATIO,
                    false);
        }
    }

    // set the turn motors's internal encoder
    @Override
    public void setTurretZero() {
        turnMotor.getInputs().position = 0;
    }

    @Override
    public double getRingRotation() {
        return turnMotor.getInputs().position / TURRET_CONSTANTS.MOTOR_TO_RING_RATIO;
    }

    @Override
    public double getRobotRelitiveRotation() {
        return getRingRotation() + TURRET_CONSTANTS.TURRET_RING_MINIMUM_TO_ROBOT_BACK_OFFSET;
    }

}
