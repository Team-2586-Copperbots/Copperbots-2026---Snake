package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.AutoLog;

import frc.robot.subsystems.intake.Intake.IntakePosition;
import frc.robot.util.auto_loggint_stuff.MotorIOInputsAutoLogged;

public interface IntakeIO {
    @AutoLog
    public static class IntakeIOInputs {

        public IntakePosition tagertPosition = null;
        public double currentCancoderPosition = 0;
    }

    // functions to control things
    public default void updateInputs() {
    }

    public default MotorIOInputsAutoLogged getMotorInputs(int id) {
        return null;
    }

    public default void setRollerSpeed(double speed) {
    }

    public default void setWristPositionTarget(IntakePosition position) {
    }

    public default void setWristPositionFromCancoder() {
    }

    public default void setWristSpeed(double speed) {
    }
}
