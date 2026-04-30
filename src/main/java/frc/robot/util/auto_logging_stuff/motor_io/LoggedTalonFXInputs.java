package frc.robot.util.auto_logging_stuff.motor_io;

import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.ControlModeValue;
import com.ctre.phoenix6.sim.TalonFXSimState;

import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.Constants;
import frc.robot.Constants.Mode;
import frc.robot.util.driveUtils.PhoenixUtil.TalonFXMotorControllerSim;

public class LoggedTalonFXInputs {
    private TalonFXInputsAutoLogged inputs;

    public LoggedTalonFXInputs() {
        inputs = new TalonFXInputsAutoLogged();
    }

    public void updateInputs(TalonFX motor) {
        // inputs.name = motor.toString();
        if (Constants.currentMode != Mode.SIM) {

            inputs.isOk = motor.isConnected();

            inputs.amps = motor.getStatorCurrent().getValueAsDouble();
            inputs.volts = motor.getMotorVoltage().getValueAsDouble();
            inputs.temp = motor.getDeviceTemp().getValueAsDouble();

            inputs.position = motor.getPosition().getValueAsDouble();
            inputs.velocity = motor.getVelocity().getValueAsDouble();

            inputs.isClosedLoop = motor.getControlMode().getValue() == ControlModeValue.PositionVoltage;
            inputs.setpoint = motor.getClosedLoopReference().getValueAsDouble();
        }
    }

    public void updateInputsSim(TalonFXSimState simState, DCMotorSim motorSim, TalonFX motor) {
        // inputs.name = motor.toString();

        inputs.isOk = true;
        Logger.recordOutput("thing/inputs volts", simState.getMotorVoltage());

        inputs.amps = simState.getTorqueCurrent();
        inputs.volts = simState.getMotorVoltage();

        inputs.position = motorSim.getAngularPositionRotations();
        inputs.velocity = motorSim.getAngularVelocityRPM() / 60;

        Logger.recordOutput("thing/control", motor.getControlMode().getValue());
        inputs.isClosedLoop = motor.getControlMode().getValue() == ControlModeValue.PositionVoltage;
        inputs.setpoint = motor.getClosedLoopReference().getValueAsDouble();

    }

    public TalonFXInputsAutoLogged getInputs(TalonFX motor) {
        updateInputs(motor);
        return inputs;
    }

    public TalonFXInputsAutoLogged getSimInputs(TalonFXSimState simState, DCMotorSim motorSim, TalonFX motor) {
        Logger.recordOutput("thing/sim outputs", true);
        updateInputsSim(simState, motorSim, motor);
        return inputs;
    }

    public void log(String dir, TalonFX motor) {
        Logger.processInputs(dir, getInputs(motor));
    }

    public void log(String dir, TalonFXSimState simState, DCMotorSim motorSim, TalonFX motor) {
        Logger.processInputs(dir, getSimInputs(simState, motorSim, motor));
    }

}
