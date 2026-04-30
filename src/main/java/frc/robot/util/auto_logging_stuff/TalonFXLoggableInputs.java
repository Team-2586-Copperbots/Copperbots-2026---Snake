package frc.robot.util.auto_logging_stuff;

import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.ControlModeValue;
import com.ctre.phoenix6.sim.TalonFXSimState;

import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.Constants;
import frc.robot.Constants.Mode;
import frc.robot.util.auto_logging_stuff.TalonFXInputsAutoLogged;
import frc.robot.util.driveUtils.PhoenixUtil.TalonFXMotorControllerSim;

public class TalonFXLoggableInputs {
    private TalonFXInputsAutoLogged inputs;
    private TalonFX motor;

    public TalonFXLoggableInputs(TalonFX motor) {
        inputs = new TalonFXInputsAutoLogged();
        this.motor = motor;
    }

    public void updateInputs() {
        // inputs.name = motor.toString();
        if (Constants.currentMode != Mode.SIM) {

            inputs.isConected = motor.isConnected();

            inputs.statorCurrent = motor.getStatorCurrent().getValueAsDouble();
            inputs.supplyCurrent = motor.getSupplyCurrent().getValueAsDouble();
            inputs.volts = motor.getMotorVoltage().getValueAsDouble();
            inputs.temp = motor.getDeviceTemp().getValueAsDouble();

            inputs.position = motor.getPosition().getValueAsDouble();
            inputs.velocity = motor.getVelocity().getValueAsDouble();

            inputs.isClosedLoop = motor.getControlMode().getValue() == ControlModeValue.PositionVoltage;
            inputs.setpoint = motor.getClosedLoopReference().getValueAsDouble();
        }
    }

    public void updateInputsSim(TalonFXSimState simState, DCMotorSim motorSim) {
        // inputs.name = motor.toString();

        inputs.isConected = true;
        Logger.recordOutput("thing/inputs volts", simState.getMotorVoltage());

        inputs.statorCurrent = simState.getTorqueCurrent();
        inputs.volts = simState.getMotorVoltage();

        inputs.position = motorSim.getAngularPositionRotations();
        inputs.velocity = motorSim.getAngularVelocityRPM() / 60;

        Logger.recordOutput("thing/control", motor.getControlMode().getValue());
        inputs.isClosedLoop = motor.getControlMode().getValue() == ControlModeValue.PositionVoltage;
        inputs.setpoint = motor.getClosedLoopReference().getValueAsDouble();

    }

    public TalonFXInputsAutoLogged getInputs() {
        updateInputs();
        return inputs;
    }

    public TalonFXInputsAutoLogged getSimInputs(TalonFXSimState simState, DCMotorSim motorSim) {
        Logger.recordOutput("thing/sim outputs", true);
        updateInputsSim(simState, motorSim);
        return inputs;
    }

    public void log(String dir) {
        Logger.processInputs(dir, getInputs());
    }

    public void log(String dir, TalonFXSimState simState, DCMotorSim motorSim) {
        Logger.processInputs(dir, getSimInputs(simState, motorSim));
    }

}
