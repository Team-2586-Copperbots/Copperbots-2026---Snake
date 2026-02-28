package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.AutoLog;

import frc.robot.Constants.IntakePosition;

public interface IntakeIO {
    @AutoLog
    public static class IntakeIOInputs {
        public double rollerSpeed = 0;
        public IntakePosition wristPosition = IntakePosition.IN;
    }

    public default void updateInputs(IntakeIOInputs inputs) {}

    public default void setRollerSpeed(double speed) {}

    public default void setWristPosition(IntakePosition position) {}
}
