package frc.robot.subsystems.climb;

import org.littletonrobotics.junction.AutoLog;

import frc.robot.subsystems.climb.Climb.ClimbPosition;
import frc.robot.util.auto_logging_stuff.TalonFXInputsAutoLogged;

public interface ClimbIO {
    @AutoLog
    public static class ClimbIOInputs {
        public boolean limitSwitch = false;

        public ClimbPosition targetPosition = ClimbPosition.DOWN;
    }

    public default void updateAndLogInputs() {
    }

    public default ClimbIOInputsAutoLogged getInputs() {
        return null;
    }

    public default TalonFXInputsAutoLogged getMotorInputs(int i) {
        return null;
    }

    public default void setPosition(double position) {
    }

    public default void setSpeed(double speed) {
    }

    public default void setTargetPosition(ClimbPosition position) {
    }
}
