package frc.robot.util.auto_logging_stuff.motor_io;

import org.littletonrobotics.junction.AutoLog;

public interface MotorIO {

    

    // AdvantageKit reads this and generates the Struct code automatically!
    @AutoLog
    public static class MotorIOInputs {
        // public String name = "";
        public boolean isOk = false;

        public double volts = 0.0;
        public double amps = 0.0;

        public double position = 0.0;
        public double velocity = 0.0;

        public boolean isClosedLoop = false;
        public double setpoint = 0.0;
    }

    // A method to actually read the motor and fill the variables
    public default void updateInputs(MotorIOInputs inputs) {

    }
}