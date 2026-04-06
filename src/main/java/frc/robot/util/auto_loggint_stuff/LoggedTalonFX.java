package frc.robot.util.auto_loggint_stuff;

import org.littletonrobotics.junction.AutoLog;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.ControlModeValue;

public class LoggedTalonFX extends TalonFX {
    public TalonFXIOInputsAutoLogged inputs;

    @AutoLog
    public static class TalonFXIOInputs {
        // public String name = "";
        public boolean isOk = false;

        public double volts = 0.0;
        public double amps = 0.0;
        public double temp = 0.0;

        public double position = 0.0;
        public double velocity = 0.0;

        public boolean isClosedLoop = false;
        public double setpoint = 0.0;
    }

    public LoggedTalonFX(int id, CANBus canbus) {
        super(id, canbus);
        inputs = new TalonFXIOInputsAutoLogged();
        updateInputs();
    }

    public LoggedTalonFX(int id) {
        this(id, new CANBus());
    }

    public LoggedTalonFX(int id, String canbus) {
        this(id, new CANBus(canbus));
    }

    public void updateInputs() {
        // inputs.name = motor.toString();

        inputs.isOk = this.isAlive();

        inputs.amps = this.getStatorCurrent().getValueAsDouble();
        inputs.volts = this.getMotorVoltage().getValueAsDouble();
        inputs.temp = this.getDeviceTemp().getValueAsDouble();

        inputs.position = this.getPosition().getValueAsDouble();
        inputs.velocity = this.getVelocity().getValueAsDouble();

        inputs.isClosedLoop = this.getControlMode().getValue() == ControlModeValue.PositionVoltage;
        inputs.setpoint = this.getClosedLoopReference().getValueAsDouble();
    }

    public TalonFXIOInputsAutoLogged getInputs() {
        updateInputs();
        return inputs;
    }

}
