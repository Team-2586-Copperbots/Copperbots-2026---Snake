package frc.robot.util.auto_loggint_stuff;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;

public class AutoConfigAndLog {
    private TalonFX motor;
    private MotorIOTalon io;
    private MotorIOInputsAutoLogged inputs;
    private String logRoot;

    public AutoConfigAndLog(TalonFX motor, String logRoot, boolean config) {
        this.motor = motor;
        this.logRoot = logRoot;


    }

    private void config() {
        TalonFXConfiguration config = new TalonFXConfiguration();
        config.CurrentLimits.StatorCurrentLimit = 90;
        motor.getConfigurator().apply(config);
    }

    private void makeIO() {
        io = new MotorIOTalon(motor);
        inputs = new MotorIOInputsAutoLogged();
    }
}
