package frc.robot.util.auto_logging_stuff;

public class SimMotorAutoLogged extends TalonFXAutoLogged {

    public SimMotorAutoLogged() {
        super(0);
        inputs.isOk = false;
        inputs.amps = -1;
        inputs.temp = -1;
        inputs.volts = -1;
    }

    @Override
    public void updateInputs(boolean forceRefresh) {
        // In simulation, do not overwrite simulated inputs with CAN signals
    }

    @Override
    public void updateInputsFromRefreshedSignals() {
        // In simulation, do not overwrite simulated inputs with CAN signals
    }

    public void setSimSpeed(double speed) {
        inputs.isClosedLoop = false;
        inputs.setpoint = speed * 100;
        inputs.velocity = speed * 100;
    }

    public void setSimTarget(double target, boolean isVelocity) {
        inputs.isClosedLoop = true;
        if (isVelocity) {
            inputs.setpoint = target;
            inputs.velocity = target;
        } else {
            inputs.setpoint = target;
            inputs.position = target;
        }
    }

}
