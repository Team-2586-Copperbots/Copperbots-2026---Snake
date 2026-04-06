package frc.robot.util.auto_loggint_stuff;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.ControlModeValue;

public class MotorIOTalon implements MotorIO {
    private TalonFX motor;

    public MotorIOTalon(TalonFX motor) {
        this.motor = motor;
    }

    @Override
    public void updateInputs(MotorIOInputs inputs) {
        // inputs.name = motor.toString();

        inputs.isOk = motor.isAlive();

        inputs.amps = motor.getStatorCurrent().getValueAsDouble();
        inputs.volts = motor.getMotorVoltage().getValueAsDouble();

        inputs.position = motor.getPosition().getValueAsDouble();
        inputs.velocity = motor.getVelocity().getValueAsDouble();

        inputs.isClosedLoop = motor.getControlMode().getValue() == ControlModeValue.PositionVoltage;
        inputs.setpoint = motor.getClosedLoopReference().getValueAsDouble();
    }

}
