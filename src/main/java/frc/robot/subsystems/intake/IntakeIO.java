package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.AutoLog;

import com.ctre.phoenix6.StatusSignal;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.intake.Intake.IntakePosition;

public interface IntakeIO {
    @AutoLog
    public static class IntakeIOInputs {
        public double rollerSetpoint = 0;
        public IntakePosition wristSetpoint = IntakePosition.IN;
        public double percentageWristSpeed = 0;
        public boolean isClosedLoop = true;

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
