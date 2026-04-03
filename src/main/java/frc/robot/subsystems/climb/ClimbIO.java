package frc.robot.subsystems.climb;

import org.littletonrobotics.junction.AutoLog;

import frc.robot.subsystems.climb.Climb.ClimbPosition;
import frc.robot.util.AutoLoggableMotorInputs;

public interface ClimbIO {
    @AutoLog
    public static class ClimbIOInputs {
        public boolean isPositionVoltage = false;
        public double motorPosition = 0;
        public boolean limitSwitch = false;
        public AutoLoggableMotorInputs folower = new AutoLoggableMotorInputs();

        public double targetSpeed = 0;
        public ClimbPosition targetPosition = ClimbPosition.DOWN;
    }

    public default void updateInputs(ClimbIOInputs inputs) {
    }

    public default void setPosition(double position) {
    }

    public default void setSpeed(double speed) {
    }

    public default void setTargetPosition(ClimbPosition position) {
    }
}
