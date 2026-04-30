package frc.robot.util.auto_logging_stuff;

public class SimMotorAutoLogged extends TalonFXAutoLogged {

    public SimMotorAutoLogged() {
        super(0);
        getInputs().isConected = false;
        getInputs().statorCurrent = -1;
        getInputs().temp = -1;
        getInputs().volts = -1;
    }

    public void setSimSpeed(double speed) {
        getInputs().isClosedLoop = false;
        getInputs().setpoint = speed * 100;
        getInputs().velocity = speed * 100;
    }

    public void setSimTarget(double target, boolean isVelocity) {
        getInputs().isClosedLoop = true;
        if (isVelocity) {
            getInputs().setpoint = target;
            getInputs().velocity = target;
        } else {
            getInputs().setpoint = target;
            getInputs().position = target;
        }
    }

}
