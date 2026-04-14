package frc.robot.subsystems.shooter;

import org.littletonrobotics.junction.Logger;

import frc.robot.util.auto_logging_stuff.SimMotorAutoLogged;
import frc.robot.util.auto_logging_stuff.TalonFXInputsAutoLogged;

public class ShooterIOSim implements ShooterIO {
    private SimMotorAutoLogged shotterMotor;

    public ShooterIOSim() {
        shotterMotor = new SimMotorAutoLogged();
    }

    @Override
    public void updateInputs() {
        Logger.processInputs("Shooter/motor1", shotterMotor.getInputs());

    }

    @Override
    public TalonFXInputsAutoLogged getMotorInputs(int id) {
        return shotterMotor.getInputs();
    }
    @Override
    public void setMotorSetpoint(double velocity) {
        shotterMotor.setSimTarget(velocity, true);
    }

    @Override
    public void setPercentageSpeed(double speed) {
        shotterMotor.setSimSpeed(speed);
    }
}
