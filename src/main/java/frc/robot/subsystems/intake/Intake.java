package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {
    private IntakeIO io;
    private IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();

    public Intake(IntakeIO io) {
        this.io = io;
    }

    // Constants.IntakePosition
    public void setIntakePosition(IntakePosition position) {
        io.setWristPosition(position);
    }

    // positive is out
    public void setMovementBarSpeed(double speed) {
        io.setWristSpeed(speed);
    }

    public void setRollerSpeed(double speed) {
        if ((inputs.wristSetpoint == IntakePosition.IN) || (inputs.currentWristPosition == IntakePosition.IN.value)) {
            io.setRollerSpeed(speed);
        }
    }

    public double getMovementBarPosition() {
        return inputs.currentWristPosition;
    }

    public boolean getIsDown() {
        double threshold = 0.05;
        if (((inputs.currentWristPosition - IntakePosition.IN.value) < threshold)
                || ((inputs.currentWristPosition - IntakePosition.OUT.value) < threshold)) {
            return true;
        } else {
            return false;
        }

    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Intake", inputs);
    }

    public static enum IntakePosition {
        IN(0.359),
        OUT(0.984),
        HALFWAY(0.5);

        public final double value;

        private IntakePosition(double value) {
            this.value = value;
        }
    }
}
