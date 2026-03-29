package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.AutoLog;

import frc.robot.subsystems.intake.Intake.IntakePosition;

public interface IntakeIO {
    @AutoLog
    public static class IntakeIOInputs {
        public boolean wristIsOK = false;
        public double wristVolts = 0;
        public double wristAmps = 0;
        public IntakePosition wristSetpoint = IntakePosition.IN;
        public boolean wristIsClosedLoop = true;

        public double rollerSetpoint = 0;

        public double currentWristPosition = 0;
        public double currentCancoderPosition = 0;
        public double currentRollerSpeed = 0;
    }

    // functions to control things
    public default void updateInputs(IntakeIOInputs inputs) {
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
