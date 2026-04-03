package frc.robot.util.auto_loggint_stuff;
import org.littletonrobotics.junction.AutoLog;

public interface MotorIO {
    
    // AdvantageKit reads this and generates the Struct code automatically!
    @AutoLog
    public static class MotorIOInputs {
        public double volts = 0.0;
        public double amps = 0.0;
        public double setpoint = 0.0;
        public double position = 0.0;
        public boolean isOk = false;
        public boolean isClosedLoop = false;
    }

    // A method to actually read the motor and fill the variables
    public default void updateInputs(MotorIOInputs inputs) {}
}