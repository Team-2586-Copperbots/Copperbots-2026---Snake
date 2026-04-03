package frc.robot.util.auto_loggint_stuff;

import static edu.wpi.first.units.Units.Rotations;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.ControlModeValue;
import edu.wpi.first.util.struct.StructSerializable;

public class AutoLoggableMotor implements StructSerializable {
    public double volts;
    public double amps;
    public double setpoint;
    public double position;
    public boolean isOk;
    public boolean isClosedLoop;

    public AutoLoggableMotor(TalonFX motor) {
        update(motor);
    }

    public AutoLoggableMotor() {
        this.volts = 0;
        this.amps = 1;
        this.setpoint = 0;
        this.position = 0;
        this.isOk = false;
        this.isClosedLoop = false;
    }

    public void update(TalonFX motor) {
        volts = motor.getMotorVoltage().getValueAsDouble();
        amps = motor.getStatorCurrent().getValueAsDouble();
        setpoint = motor.getClosedLoopReference().getValueAsDouble();
        position = motor.getPosition().getValue().in(Rotations);
        isOk = motor.isAlive();
        isClosedLoop = motor.getControlMode().getValue() != ControlModeValue.DutyCycleOut;
    }

    // The Struct definition
    public static final AutoLoggableMotorStruct struct = new AutoLoggableMotorStruct();

}
