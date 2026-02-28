package frc.robot.subsystems.climb;

import org.littletonrobotics.junction.AutoLog;

import frc.robot.subsystems.climb.Climb.ClimbPosition;

public interface ClimbIO {
    @AutoLog
    public static class ClimbIOInputs {
        public boolean positionVoltage = false;
        public double speed = 0;
        public ClimbPosition position = ClimbPosition.DOWN;
    }

    public default void updateInputs(ClimbIOInputs inputs) {
    }

    public default void setSpeed(double speed) {
    }

    public default void setPosition(ClimbPosition position) {
    }
}
