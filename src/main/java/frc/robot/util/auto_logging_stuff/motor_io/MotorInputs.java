package frc.robot.util.auto_logging_stuff.motor_io;

import org.littletonrobotics.junction.AutoLog;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.ControlModeValue;

// AdvantageKit reads this and generates the Struct code automatically!
@AutoLog
public class MotorInputs {
    // public String name;
    public boolean isOk;

    public double volts;
    public double amps;

    public double position;
    public double velocity;

    public boolean isClosedLoop;
    public double setpoint;

    public MotorInputs() {
        // public String name = "";
        isOk = false;

        volts = 0.0;
        amps = 0.0;

        position = 0.0;
        velocity = 0.0;

        isClosedLoop = false;
        setpoint = 0.0;
    }

    public void updateInputs(TalonFX motor) {
        // inputs.name = motor.toString();

        isOk = motor.isAlive();

        amps = motor.getStatorCurrent().getValueAsDouble();
        volts = motor.getMotorVoltage().getValueAsDouble();

        position = motor.getPosition().getValueAsDouble();
        velocity = motor.getVelocity().getValueAsDouble();

        isClosedLoop = motor.getControlMode().getValue() == ControlModeValue.PositionVoltage;
        setpoint = motor.getClosedLoopReference().getValueAsDouble();
    }
}