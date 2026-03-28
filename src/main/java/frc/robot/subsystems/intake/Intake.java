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
    public void setIntakePositionTarget(IntakePosition position) {
        io.setWristPositionTarget(position);
    }

    // positive is out
    public void setWristSpeed(double speed) {
        io.setWristSpeed(speed);
    }

    public void setRollerSpeed(double speed) {
        double distanceToStopAt = 0.4;
        if ((Math.abs(inputs.currentWristPosition - IntakePosition.IN.value)) < distanceToStopAt) {
            io.setRollerSpeed(0);
        } else {
            io.setRollerSpeed(speed);
        }
    }

    public double getWristPosition() {
        return inputs.currentWristPosition;
    }

    public IntakePosition getWristTarget() {
        return inputs.wristSetpoint;
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

    // public boolean isAtTarget() {
    //     double tolerence = 0.05;
    //     if (Math.abs(getWristPosition() - IntakePosition.HALFWAY.value) < tolerence) {
    //         return true;
    //     } else {
    //         return false;
    //     }
        
    // }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Intake", inputs);
    }

    // public void refreshPosition() {
    //     io.setWristPositionFromCancoder();
    // }

    public static enum IntakePosition {
        IN(0.0),
        OUT(0.245);
        // ,
        // HALFWAY(4);

        public final double value;

        private IntakePosition(double value) {
            this.value = value;
        }
    }
}
