package frc.robot.subsystems.climb;

import org.littletonrobotics.junction.Logger;

import frc.robot.subsystems.climb.Climb.ClimbPosition;
import frc.robot.util.auto_logging_stuff.SimMotorAutoLogged;
import frc.robot.util.auto_logging_stuff.TalonFXInputsAutoLogged;

public class ClimbIOSim implements ClimbIO {
    private SimMotorAutoLogged climbMotor1;
    private ClimbIOInputsAutoLogged inputs;

    public ClimbIOSim() {
        climbMotor1 = new SimMotorAutoLogged();
        inputs = new ClimbIOInputsAutoLogged();
        inputs.limitSwitch = true;
    }

    @Override
    public void updateAndLogInputs() {
        Logger.processInputs("Climb/motor", climbMotor1.getInputs());

    }

    @Override
    public ClimbIOInputsAutoLogged getInputs() {
        return inputs;
    }

    @Override
    public TalonFXInputsAutoLogged getMotorInputs(int id) {
        return climbMotor1.getInputs();
    }

    @Override
    public void setSpeed(double speed) {
        climbMotor1.setSimSpeed(speed);
    }

    @Override
    public void setTargetPosition(ClimbPosition position) {
        climbMotor1.setSimTarget(position.value, false);
    }
}
