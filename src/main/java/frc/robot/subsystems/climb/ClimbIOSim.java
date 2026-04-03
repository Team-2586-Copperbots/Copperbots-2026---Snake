package frc.robot.subsystems.climb;

import static frc.robot.Constants.CANIds.Canivore;

import org.ironmaple.simulation.motorsims.SimulatedMotorController;

import com.ctre.phoenix6.hardware.TalonFX;

import frc.robot.Constants.CANIds;
import frc.robot.subsystems.climb.Climb.ClimbPosition;

public class ClimbIOSim implements ClimbIO {
    private Double targetSpeed = 0.0;
    private ClimbPosition position = ClimbPosition.DOWN;
    private boolean positionVoltage = false;
    private TalonFX motor = new TalonFX(CANIds.CLIMB_MOTOR_1, Canivore);

    public ClimbIOSim() {

    }

    @Override
    public void updateInputs(ClimbIOInputs inputs) {
        inputs.motorPosition = position.value;
        inputs.isPositionVoltage = positionVoltage;
        // inputs.folower.update(motor);

        inputs.targetPosition = position;
        inputs.targetSpeed = targetSpeed;

    }

    @Override
    public void setSpeed(double speed) {
        this.targetSpeed = speed;
        this.positionVoltage = false;
    }

    @Override
    public void setTargetPosition(ClimbPosition position) {
        this.position = position;
        this.positionVoltage = true;
    }
}
